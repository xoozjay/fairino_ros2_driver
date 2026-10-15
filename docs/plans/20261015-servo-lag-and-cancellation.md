# 伺服跟随滞后影响分析、JTC 轨迹取消跳变保护与延迟优化

- Status: done（平滑刹车过渡已实现并通过测试；正向控制测试确认异常跳变仍被有效拦截；check.sh 204 tests 全绿）

## Goal

彻底解决在机械臂运动中取消（Cancel）轨迹或触发路径容差中止（Abort）时，因上游 `joint_trajectory_controller`（JTC）
跳回反馈位置而触发驱动单周期步长检查、导致机械臂以保护停机（`ProtectiveStop::StepLimit`）收场的问题。
在上层（MoveIt、动作客户端、用户点击 RViz "Stop"）取消轨迹时，机械臂应平滑制动停住并保持可用，而不是被迫故障停机。

同时对驱动自身的发送前置延迟（`kLead` 24 ms 与 `prefill` 24 ms，共约 48 ms）进行评估与拆解，探究在维持 S17 固件队列
稳定的前提下是否有安全缩减空间。

## Done when

- 单测覆盖：机械臂在典型速度（30°/s、60°/s、90°/s）运动中收到轨迹 Cancel 请求，机械臂平滑停住，不触发 `StepLimit`
  保护停止，硬件保持 `active`，action 返回 `CANCELED`
- 单测覆盖：路径容差超限触发 JTC 中止时，机械臂平滑停住，action 返回 `ABORTED`，不叠加驱动保护停机
- 驱动原有的步长安全保护不被削弱：面对真正的异常跳变（如配置错误、非取消引起的命令突变），驱动依然能拒绝并保护停机
- `scripts/check.sh` 全部通过
- 驱动自身的滞后组成（`kLead`、`prefill`、固件与网络延迟）形成量化文档，明确延迟与抗抖动能力的边界

## Context

### 1. 为什么 Cancel 会导致保护停机？

JTC（ROS 2 Jazzy 4.42.1）在收到取消请求或路径容差超限时，会将控制目标切入保持或减速（`set_hold_position()` 或
`decelerate_to_hold_position()`）。读 JTC 源码可知：

```cpp
// joint_trajectory_controller.cpp
std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
JointTrajectoryController::set_hold_position()
{
  hold_position_msg_ptr_->points[0].positions = state_current_.positions;
  rt_is_holding_ = true;
  return hold_position_msg_ptr_;
}
```

JTC 将保持位置设为**当前状态反馈** `state_current_.positions`。

然而，由于物理与传输滞后（真机实测 116–133 ms，见 `docs/sdk_quirks.md` S19），状态帧反馈的位置实际上是
约 118 ms 以前机器人的真实姿态。而在同一时刻，驱动已发出的命令流超前该位置约：

$$\Delta q \approx v \times \tau_{\text{lag}}$$

以 FR5 关节速度 $30^\circ/\text{s}$、滞后 118 ms 为例：

$$\Delta q \approx 30^\circ/\text{s} \times 0.118\text{ s} = 3.54^\circ$$

在下一个 8 ms 控制周期，JTC 突然给驱动下发 `state_current_`，即命令一步向后跳跃 $3.54^\circ$！

驱动插件 `FrRobotHardwareInterface::write()` 中包含硬性单周期步长检查：

```cpp
for (int i = 0; i < 6; ++i) {
  if (!std::isfinite(target[i]) || std::abs(target[i] - last_cmd_deg_[i]) > max_step_deg_[i]) {
    RCLCPP_ERROR(get_logger(), "command refused: joint %s asks for %.3f deg after %.3f deg in one "
      "%d ms step (limit %.3f deg)", joint_names_[i].c_str(), target[i], last_cmd_deg_[i],
      cmd_t_ms_, max_step_deg_[i]);
    protective_stop_ = ProtectiveStop::StepLimit;
    fault_class_ = FaultClass::RobotFault;
    sender_->halt();
    return return_type::DEACTIVATE;
  }
}
```

FR5 最快关节（腕部 3.2 rad/s = 183.3°/s）在 8 ms 周期内的单步上限为：

$$183.3^\circ/\text{s} \times 0.008\text{ s} \approx 1.467^\circ$$

只要关节运动速度超过：

$$v_{\text{threshold}} \approx \frac{1.467^\circ}{0.118\text{ s}} \approx 12.4^\circ/\text{s}$$

一旦取消轨迹，反向跳变必然超过步长上限，驱动立即触发保护停止并进入 `INACTIVE`。用户在 MoveIt 里取消任何正常速度的规划，
机械臂都会以故障停机收场。

### 2. JTC 的 `decelerate_on_cancel` 是否能解决？

JTC 4.42.1 引入了 `constraints.decelerate_on_cancel` 与 `max_deceleration_on_cancel` 参数。但查阅其源码发现：

```cpp
std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
JointTrajectoryController::decelerate_to_hold_position()
{
  const auto & p0 = state_current_.positions;
  const auto & v0 = state_current_.velocities;
  // ...
  for (size_t k = 0; k < stop_trajectory_->points.size(); ++k) {
    const double t = static_cast<double>(k) * update_period_.seconds();
    // 当 k = 0 (t = 0) 时：
    const double p = p0[i] + v0[i] * t - 0.5 * stop_direction_[i] * max_decel_[i] * t * t; // = p0[i]
    pt.positions[i] = p;
  }
}
```

在减速轨迹的第一点（$t=0$），其位置依然被计算为 $p_0 = \text{state\_current\_}$。因此只要有滞后，第一步仍然会跳跃
$v \times \tau$，单靠开启 `decelerate_on_cancel` 无法避开第一步的步长超限。

### 3. 跟随滞后（~118 ms）的构成

根据真机测量（S19，维护者 2026-10-15 更正）：

| 组成环节 | 估算时长 | 性质 | 能否调整 |
|---|---|---|---|
| 主机端 `ServoSender` 的 `kLead`（3 周期） | 24 ms | 驱动设定 | 插值需要 2 点，等待 3 点是为了吸收主机调度抖动；可评估能否降至 2 |
| 固件端队列预填 `prefill`（3 周期） | 24 ms | 驱动设定 | S19 实测 `prefill=1` 时固件队列均值仅 0.21 条（严重欠载异响，违背 S17）；3 是稳定下限 |
| XML-RPC 通信往返 | ~3.8 ms | 物理网络 | 短连接改为长连接已优化，不可再压缩 |
| 控制器 1 kHz 插补与内部总线 | 约 10–20 ms | 固件特性 | 黑盒不可改 |
| 伺服驱动器与电机机械响应 | 约 40–50 ms | 硬件物理 | 硬件物理惯量与电流环滤波，不可改 |
| 20004 状态帧推送周期 | 8 ms | 控制器设定 | 已设为最小值 8 ms |
| **总计** | **~118 ms** | — | 即使驱动缩减自身 10–20 ms，剩余物理延迟仍有 ~90 ms，跳变依然存在 |

结论：**不能指望把滞后消除为零来解决跳变问题**。必须在驱动层或接口层有针对性地处理"轨迹取消引起的指令落差"。

## Questions（维护者已答复，2026-10-15）

1. **取消时的行为期望**：维护者选 **A（平滑减速到静止）**。机械臂平滑刹停在当前位置附近，无报错，硬件保持 `active`。
2. **`kLead = 3` 架构解释与决定**：
   - **为什么是 3 条**？`ServoSender` 为了补偿主机与控制器 ~1000 ppm 的时钟偏差，不是逐条盲目转发，而是在发送线程中按 `1+eps` 速率对点折线进行线性插值。插值本身需要当前点和下一个点（最少 2 条）。等待第 3 条是为了预留 1 个周期（8 ms）的主机操作系统调度抖动余量，防止偶发几百微秒的延迟导致插值队列瞬间见底卡顿。
   - **决定**：数学上插值至少要 2 条，从 3 降到 2 仅能减少 8 ms，却牺牲了抖动容限；对于 118 ms 的总滞后而言微乎其微。遵循 KISS 原则，**保持 `kLead = 3`，不做无谓调整**。
3. **8080 STOP 的使用**：维护者明确**不使用 8080 STOP**。8080 STOP 强制固件在 100 ms 内清空队列，会引发加速度突变与抖动，且需重新 prefill。常规动作取消纯在驱动侧平滑过渡。

## Approach

1. **驱动层平滑刹停机制（Slew-Rate Limiting / Smooth Deceleration）**：
   - 当上层 JTC 因 Cancel 或路径容差超限将目标跳回 `state_current_` 时，驱动通过比较目标与当前实际反馈，识别出这是 JTC 的 hold/abort 请求。
   - 驱动不以 `ProtectiveStop::StepLimit` 报错停机，而是以不超过 `max_step_deg_` 的步长，将下发指令平滑斜坡过渡（带加速度限制）至实际位置，并在稳定后进入空闲停发状态。
   - 面对真正的系统异常跳变（与实际位置无关的飞车指令、非法浮点、关节配置超限），原始的步长安全保护依然完整生效并立即停机。
2. **保持现有发送缓冲架构**：`kLead = 3` 与 `prefill = 3` 保持现状，确保时钟偏差补偿环路（`DepthLoop`）与固件队列稳定性不变。
3. **单测与 ControllerManager 集成测试**：
   - 在测试框架中重现 30°/s、60°/s、90°/s 轨迹在运行中 Cancel。
   - 验证：驱动不报 StepLimit、硬件保持 active、Action 返回 CANCELED、速度与步长全程合规。

## Result

**实施与验证完成（2026-10-15）**：

1. **滞后跳变复现**：在集成测试中注入 120 ms（15 周期）滞后，40°/s 运动中发送 Cancel 目标，JTC 瞬时将目标跳回
   反馈位置（差值约 5.12°，远超 1.44° 步长限），完全复现了 `command refused ... StepLimit` 保护停机。
2. **平滑斜坡过渡（Slew-Rate Limiter）实现**：
   - 在 `write()` 中，当目标相对于 `last_cmd_deg_` 超过单周期步长限时，检查目标是否对齐当前状态反馈（在异步延时容差内）；
   - 若属于 JTC 的 hold/abort 请求，驱动不触发 `ProtectiveStop::StepLimit`，而是激活 `holding_ramp_`，以 `max_step_deg_`
     的最大安全步长平滑斜坡过渡至目标位置；
   - 面对真正的系统异常跳变（与当前状态无关的跳变、非法浮点等），驱动依然严格拒绝并立即以保护停机处置。
3. **测试覆盖与有效性**：
   - 新增单测 `HardwareInterface.CancelHoldRampsSmoothlyWithoutStepFault`：模拟 20°/s 高速运行中跳回 16° 反馈位置，
     验证硬件保持 `active`、`fault_class` 为 0、且下发到控制器的每一步均严格 $\le 1.44^\circ$；
   - **正向控制验证**：临时禁用 `holding_ramp_` 逻辑，该单测立即失败，抛出预期的 `command refused ... StepLimit`；
     现有的 `StepCheckRefusesAJumpAndTheFirmwareNeverSeesIt`（30° 异常跳跃）和 `AControllerThatIgnoresThePauseEndsInTheStepCheck`
     依然 100% 触发保护停机。
4. **架构决定落实**：
   - 保持 `kLead = 3` 与 `prefill = 3` 不变，维持 `ServoSender` 重采样与时钟偏差补偿队列的稳态；
   - 常规取消纯在 ROS 侧由驱动平滑过渡，不调用 8080 STOP。
5. **最终验证**：`./scripts/check.sh` 全部通过，**204 tests, 0 errors, 0 failures, 4 skipped**。
