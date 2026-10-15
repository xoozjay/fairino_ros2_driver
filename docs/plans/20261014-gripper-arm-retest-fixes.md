# 让夹爪不再干扰机械臂：轨迹暂停代替扣臂停机，修正夹爪完成判定与过期命令

- Status: done（D 组由维护者取消：夹爪命令不与机械臂命令同时发，不这样测；驱动的暂停只作兜底）

## Goal

[20261013-pre-merge-cleanup.md](20261013-pre-merge-cleanup.md) 的真机复测（2026-10-14，3.8.4.2，FR5，PGI-140，
J6 ±30°）发现：夹住物体后机械臂与夹爪被锁 30 s；被中止的夹爪目标十几秒后仍会执行；扣臂机制依赖的路径容差按
未实测的 40 ms 滞后取值，实际滞后 118 ms；夹爪运动中收到机械臂轨迹，驱动会保护停止机械臂。合入前完成（维护者：main 应当是直接可用的驱动）。

原则（维护者）：**外设不应干扰本体**。夹爪最多让机械臂的轨迹推迟，不得让机械臂停机或轨迹失败。控制器本身不允许
`MoveGripper` 与 `ServoJ` 同时执行（G8、G9），所以二者只能轮流动；轮流由驱动安排，对上层表现为轨迹暂停、夹爪等待。

## Done when

- 下文"测试"一节：A、B 组进入 `scripts/check.sh` 并通过；C 组手动通过（D 组已取消，见 Status）
- 驱动代码中不再有因夹爪而起的保护停止（`ProtectiveStop::HoldTimeout` 删除）
- sdk_quirks 增加本次实测（下文 E1–E5），README 夹爪段与故障表同步

## Context

真机会话日志在会话机器的 `/tmp/session/`（不入库）；以下为关键数据。

**E1 跟随滞后约 118 ms，不随速度变化**（维护者更正 2026-10-15：这个常数既有网络与固件的份额，也有本驱动自己的份额——主机端 `kLead` 3 条与固件队列 `prefill` 3 条共约 48 ms 都是驱动的设定）。J6 余弦轨迹，`joint_trajectory_controller/controller_state` 的 reference 与
feedback 互相关：30°/s 最大误差 3.7°、60°/s 7.4°、90°/s 11.1°，滞后均为 116–121 ms（含 +2°/4 s、+10°/5 s 的慢轨迹）。
`controllers.yaml:44-45` 的路径容差 0.26 rad 按"prefill × cmdT + 往返，约 40 ms"算出，此值从未实测。按 118 ms，
0.26 rad 在约 126°/s 以上误报 ABORTED；误报后 JTC 保持在实际位置，命令一步跳回约 15°，单周期步长检查拒绝 →
高速中保护停止。未在真机上跑 > 90°/s。滞后的组成：驱动自身——主机端 `kLead` 3 条（24 ms）与固件队列
`prefill` 3 条（24 ms）；网络与固件——XML-RPC 往返、控制器插补与总线、伺服与机械响应、状态帧采样，其余约 70 ms 未拆分。

**E2 夹爪运动中发机械臂轨迹 → 保护停止**（维护者 2026-10-15：夹爪运动时不应有机械臂运动；这是上层的误用，驱动只需兜底，不让本体停机）。夹爪 100 → 50 % 期间发 J6 +2°/4 s：被扣住；参考只偏 2°，到不了路径容差；
夹爪结束后 2 s 内命令未回到停住位置 → `HoldTimeout` 保护停止（`fr_robot_hardware_interface.cpp:790`）。
夹爪计划 Result 中同一操作的记录（goal_time 中止，之后同一轨迹成功）不再成立。大幅轨迹（0 → +30°，30°/s）则按设计
因路径容差 ABORTED、放行，无保护停止。

**E3 夹住物体时 `gripper_motiondone = 2`**。夹爪 50 % 处放物体后命令 0：位置 50 → 45 → 23 → 7 %，帧报 `motion_done = 2`
并保持。SDK 文档只写 0/1（`references/fr_documentation/source/SDKManual/CPPDataStructure.rst:204`）。**寄存器状态位**（维护者结论 2026-10-15，sdk_quirks G13）：两个标志位，**0x02 卡住**（运动中受阻）、**0x01 到达**（到目标）；新命令开始时清零，**只在命令期间（运动时限内）改变**，之后保持不变。故 0 = 运动中，1 = 到达，2 = 卡住（夹着物体停下），3 = 先卡住、后到达（运动中物体被抽走）。时限过后物体掉落，状态位不变，不可观测。驱动只认 1 为完成
（`fr_robot_hardware_interface.cpp:872-874`），于是 `gripper_busy` 保持到 30 s 超时：期间机械臂命令被扣、新的夹爪
命令（含张开）也不发；张开目标因 `stall_timeout` 2 s ABORTED。

**E4 被中止的夹爪目标之后仍执行**。`parallel_gripper_action_controller` 因 `stall_timeout` 中止目标后，命令接口里的目标值
不变；驱动在条件满足时（机械臂停下、或 busy 超时）照发。两次实测：机械臂先动、夹爪目标 ABORTED，机械臂停后夹爪
张到 100 %；夹住物体时张开目标 ABORTED，约 15 s 后 busy 超时，夹爪自行张到 50 %、松开物体（停用 `gripper_controller`
晚了约 0.1 s）。

**E5 原地 `MoveGripper`**（驱动停止，帧级 1 ms 轮询，无物体，100 % → 100 %、50 % → 50 %）：调用后第一帧 `motion_done = 0`，
0.66–0.70 s 回到 1，其后位置仍报 0 % 约 200 ms 才恢复。驱动只在 `motion_done == 0` 时保持开度
（`fr_robot_hardware_interface.cpp:846`），这 200 ms 对外报开度 0。夹住物体时原地命令（7 % → 7 %）没有这个现象：位置先于
完成信号恢复，完成信号为 2。正常移动（100 → 50 %）：0 立即出现，约 0.8 s 起位置每 0.4 s 更新，2.24 s 完成（与 G10 一致）。
回答了前一计划的问题："到当前开度时 0 是否出现"——出现，1.5 s 的"未开始"兜底在这些情况下不触发。

**其他**：保护停止后 `~/status` 的 `arm_commands` 仍报 `refused (gripper busy)`（`status_report.hpp:216` 的 `arm_held`
未随停用清除）。

**JTC 的速度缩放**（上游 4.42.1 `joint_trajectory_controller.cpp` 的 `update()`，读源码）：每周期从状态接口读因子；
新轨迹首次采样时 `traj_time_ = time`，之后 `traj_time_ += period * scaling_factor_`，因子为 0 时参考停在轨迹起点；
发给硬件的命令取 `sample(traj_time_ + update_period_)`，冻结时比参考超前一个周期（线性插值、30°/s：真机 125 Hz 时 0.24°，mock 500 Hz 时 0.06°）；
恢复的那个周期命令取 `sample(traj_time_ + 2 × update_period_)`，比停住位置超前两个周期；
路径容差比较 `traj_time_` 处的参考与反馈，`goal_time` 的 `time_difference` 也按 `traj_time_` 算，暂停不计入；
取消与中止时 `set_hold_position()` 保持在实际位置（`state_current_.positions`）。

## Approach

1. **完成判定与夹爪 action**（此前"合入后"的计划提前；维护者选 A）：夹爪 action 改由驱动插件的节点提供，类型仍为
   `control_msgs/action/ParallelGripperCommand`，替代 `parallel_gripper_action_controller`。原因：后者只看开度速度
   （上游 `parallel_gripper_action_controller_impl.hpp` 的 `check_for_success`），分不清"夹住物体"与"在等机械臂停下"
   （后者也会在 `stall_timeout` 后被判为卡住），且中止/成功都把目标留在命令接口里（E4 的根因；`accepted_callback`
   只写实名，不撤旧值）。做法：
   - 完成判定直接用帧："先报 0、再报非 0"结束——报 1 到位，action SUCCEEDED；报 2 夹住，SUCCEEDED 且
     `stalled = true`（维护者确认）；报 3 掉落，ABORTED；等机械臂停下的目标继续等，不算卡住（机械臂优先）；
     发出 30 s 仍无完成的按超时 ABORTED。1.5 s"从未开始"兜底保留（E5：起始的 0 一定出现，兜底只是防线）
   - 新目标被接受时旧目标立即 ABORTED（与 JTC 的抢占一致；上游夹爪控制器为 CANCELED，但服务端抢占不经客户端取消
     请求）；旧目标已发出的运动走完后再发新目标。取消时若尚未发出则干净取消，已发出则报取消、运动让其走完
     （控制器无法中止已发出的夹爪运动）
   - 取代武装机制的保护：等待中或运动中的目标遇到停用、机器人故障、夹爪不可用、`~/gripper/activate` 即 ABORTED，
     其值不再保留；硬件未激活、夹爪未激活或报故障时拒绝新目标
   - 到当前开度的目标也照发（再次夹紧是合理用法；E5 的短暂 0 不外泄由第 3 步保证）
   - 真机管线不再加载 `gripper_controller`（launch 只在 mock 下加载）；插件节点在 `<ns>` 下，以相对名
     `gripper_controller/gripper_cmd` 创建 action（维护者选定，不做重映射），解析为 `/<ns>/gripper_controller/gripper_cmd`
     ——mock（仍用上游控制器）与真机对上层同名同类型
   - 武装机制删除（`~/status` 的 `gripper_commands` 随之删除）：action 命令只来自被接受的目标；
     `~/status` 增加 `gripper_object`（`none` / `held` / `dropped`，按 2/3）
2. **过期的夹爪命令**：随第 1 步消失——命令只来自被接受的目标，目标被取代或取消后驱动不再持有其值，上游控制器
   "中止后命令仍留在接口里"的行为不再有机会发生
3. **原地运动的开度**：发出的目标非 0 时，帧的 0 % 只在出现过非 0 位置之后才采信；否则保持运动前的开度
4. **轨迹暂停代替扣臂**（JTC 速度缩放）
   - 状态接口 `<tf_prefix>speed_scaling/speed_scaling_factor`（与 UR 驱动同名）在 xacro 里用
     `<gpio name="${tf_prefix}speed_scaling">` 声明一次、`initial_value` 1.0，真机与 mock 共用（同 `fr_status`、UR；
     维护者选定）；真机插件写值：平时 1.0，夹爪忙时（运动：从发出
     `MoveGripper` 到帧报完成；激活：激活服务执行期间）0.0。`joint_trajectory_controller` 配 `speed_scaling.state_interface` 指向它（本机 Jazzy 的
     JTC 4.42.1 已支持，`joint_trajectory_controller_parameters.hpp:91`），因子为 0 时轨迹时间不前进：参考停在原地，
     不偏离、不触发路径容差，夹爪完成后从原处继续。mock 恒为 1.0
   - 夹爪只在机械臂静止时发出（现有：伺服流停发且队列空），所以因子降为 0 时机械臂本来就停着，不会急停
   - 删除 2 s 扣臂超时、`ProtectiveStop::HoldTimeout` 与"回到停住位置才放行"（`arm_held_`）。夹爪忙时仍不把会动的
     命令发给控制器（控制器也不会执行，G9）；夹爪结束由帧确定，同一周期因子回到 1、命令即放行，只经现有的单周期
     步长检查——JTC 冻结时命令只超前停住位置一个周期，远小于步长上限。不认速度缩放的控制器属于误用，README 写明，
     由步长检查兜底。`~/status` 的 `arm_commands` 保持键名不变，值增加 `paused (gripper busy)`
   - JTC 在因子 0 时的行为以源码为准（见 Context），不另写测试确认
   - 路径容差按实测滞后的最大值重算：最快关节 3.2 rad/s × 0.133 s × 2 ≈ 0.85 rad（滞后 116–133 ms，E1 与 X3）；它只负责
     真正的跟随失败
   - 118 ms 的组成（已知 `kLead` 24 ms、`prefill` 24 ms，其余约 70 ms）用 `fr_servo_bench` 测清，减少滞后另立计划
   - 机械臂运动期间收到的夹爪目标照旧等机械臂停下（G8），不设等待时限；调用方可取消
5. **状态**：停用时清除暂停状态（因子回到 1）
6. **文档**：sdk_quirks 增加 E1（S19）、E3（G13）、E4 的驱动侧对策、E5（G14）；README 夹爪段写明夹住物体时的行为与
   SUCCEEDED/`stalled`；故障表与路径容差注释更新

**考虑过、未采纳**：
- 保持"扣住 + 2 s 保护停止"，或夹爪忙时收到会动的命令立即保护停止：违反"外设不干扰本体"
- 夹爪结束后要求命令回到停住位置（0.01°）再放行：JTC 冻结时命令超前一个周期，恢复后接着前进、回不到门槛内，
  轨迹最终被路径容差中止
- 路径容差放大或设为 0 来减少误报：被扣住的轨迹仍只剩保护停止，且失去对真正跟随失败的检测
- 不经运动队列控制夹爪、让二者真正同时动（末端 485 寄存器写 `AxleSensorRegWrite`，或夹爪 Modbus 直连主机）：
  未验证，可能与控制器对夹爪的轮询冲突，单列 TODO

MoveIt 的执行时长监控会把暂停的时间算作超时：MoveIt 计划须相应配置（该计划已把"夹爪 goal 的执行时长监控"列为前置
条件，机械臂轨迹同理）。

**风险**：夹住后物体掉落时，夹爪在没有指令的情况下自行继续闭合约 0.4–0.6 s（帧报 0，之后报 3；X2），驱动不视其为
夹爪忙；这期间机械臂若在运动，G9（夹爪运动期间 `ServoJ` 不执行）是否适用未测；运动时限过后标志位不再更新（G13⑤），
此后物体掉落不可观测（没有别的传感器）；夹爪 action 的目标回调与结果发布共用插件节点的 executor 线程，
不阻塞实时循环（参考 JTC：实时侧只用无锁缓冲）；mock 仍用上游夹爪控制器，其卡住按开度速度判定、语义与真机不同
（README 写明）；`gripper_controller` 的 `max_velocity`/`max_effort` 命令接口保留（mock 需要），真机上无人占用；
暂停中开始、只给位置且起步即高速的轨迹（线性插值，速度超过关节限速的一半），恢复时第一步是两个周期的位移，超过单周期
步长上限即保护停止；MoveIt 的轨迹从静止起步，不受影响。JTC 速度缩放在 Jazzy 是较新的功能，行为以 4.42.1 源码为准（Context），升级 JTC 时须重读。

## 测试

不收等待超时的用例：测试里的轨迹与夹爪运动都在亚秒级。

**假控制器**（`test/fake_controller.hpp`）需扩展：夹爪可在指定开度停下并报 `motion_done = 2`（夹住），之后可改报 3
（掉落）；原地运动按 E5 的序列（先报 0、未完成，完成后位置再报 0 约 200 ms）。

**A. 插件 + 假控制器**（现有 `test_hardware_interface.cpp` 的方式：ResourceManager + executor）

| # | 场景 | 期望 | 来源 |
|---|---|---|---|
| A1 | 目标运动，帧 0 → 1 | SUCCEEDED，`stalled = false`；busy 清零 | — |
| A2 | 目标运动，帧 0 → 2（夹住） | SUCCEEDED，`stalled = true`；busy 在报 2 时清零；`gripper_object = held`；随即发张开目标，夹爪张开 | E3 |
| A3 | 夹住后帧改报 3 | `gripper_object = dropped`；运动中报 3 则目标 ABORTED | E3 |
| A4 | 机械臂运动中收到目标 | 不发 `MoveGripper`、目标保持执行中；机械臂停下后发出并 SUCCEEDED | G8 |
| A5 | 等待中收到新目标；运动中收到新目标 | 新目标被接受时旧目标 ABORTED；前者只发新目标，后者当前运动走完后再发新目标 | 1 |
| A6 | 取消未发出的目标；取消已发出的目标 | 前者：CANCELED，`MoveGripper` 计数 0；后者：CANCELED，运动走完，之后不再发任何命令 | E4 |
| A7 | 等待中的目标遇到停用、机器人故障、夹爪不可用、`~/gripper/activate` | 目标 ABORTED；恢复/重新激活后夹爪不动 | 1 |
| A8 | 硬件未激活、夹爪未激活或报故障时发目标 | 目标被拒绝 | 1 |
| A10 | 目标等于当前开度 | 照发 `MoveGripper` | 1 |
| A11 | 原地运动（非 0 目标）按 E5 序列 | 对外开度全程不报 0；目标为 0 时照报 0；下一次运动或激活后不再沿用旧开度 | E5 |
| A12 | 暂停接口 | `speed_scaling_factor` 平时 1.0；从发出 `MoveGripper` 到帧报完成（1 或 2）为 0.0；激活夹爪期间 0.0；停用后 1.0 | 4、5 |
| A13 | 夹爪运动中命令变化 | 不发给控制器（`servoj_while_gripper_moves == 0`）、不保护停止、不停用；`arm_commands = paused (gripper busy)`；帧报夹爪完成的同一周期起命令放行 | E2、4 |
| A14 | 保护停止或停用后读 `~/status` | `arm_commands` 不再报暂停；无 `gripper_commands` 键；有 `gripper_object` | 其他、1 |
| A15 | action 名 | 解析为 `/<ns>/gripper_controller/gripper_cmd`，类型 `ParallelGripperCommand` | 1 |

**B. ControllerManager + JTC + 插件 + 假控制器**（新的测试装置；验证插件与 JTC 的配合，JTC 自身行为见 Context）

| # | 场景 | 期望 | 来源 |
|---|---|---|---|
| B1 | 夹爪运动中发 J6 +2°/0.5 s | 暂停期间 `controller_state` 的 reference 不变、无路径容差中止；夹爪完成后继续，SUCCEEDED | E2 |
| B2 | 夹爪运动中发只给位置的单点目标（J6 +15°，0.5 s，线性插值 30°/s） | 同 B1；恢复后无保护停止（冻结时命令超前一个周期） | 4 |
| B5 | 轨迹运行中发夹爪目标 | 夹爪等轨迹结束才发；轨迹 SUCCEEDED，目标 SUCCEEDED | G8 |
| B6 | 暂停中取消轨迹 | CANCELED；夹爪完成后机械臂不动、无保护停止 | 4 |

**C. mock 管线**（手动）：`ros2 launch fr_robot_driver fr_control.launch.py gripper:=pgi140` 后，JTC 为 active；
`speed_scaling/speed_scaling_factor` 存在且为 1.0；`ros2 action list -t` 列出 `/gripper_controller/gripper_cmd`
`[control_msgs/action/ParallelGripperCommand]`，开合各一次 SUCCEEDED。

**D. 真机**（**已取消**，维护者 2026-10-15：夹爪命令不应与机械臂命令一起发，不这样测试）

| # | 场景 | 期望 | 来源 |
|---|---|---|---|
| D1 | 夹爪 100 → 50 % 期间发 J6 +2°/4 s | 暂停、夹爪完成后继续，SUCCEEDED，无保护停止 | E2 |
| D2 | 夹爪运动期间发 0 → +30°（30°/s） | 同 D1 | E2、4 |
| D3 | 50 % 处放物体后命令 0 | 报 2 时 `gripper_busy` 清零；SUCCEEDED、`stalled = true`、`gripper_object = held`；随即张开成功 | E3 |
| D4 | 机械臂运动中发夹爪目标后取消；夹住时发张开目标后再发新目标取代 | 机械臂停下后夹爪不动；只执行最新目标 | E4 |
| D5 | 原地 `MoveGripper`（100 → 100 %、50 → 50 %） | `joint_states` 的开度不报 0 | E5 |
| D6 | J6 余弦轨迹，最高 180°/s | SUCCEEDED，不因路径容差中止；记录最大误差与滞后 | E1 |
| D7 | `fr_servo_bench` 测 118 ms 的组成 | 只记录数据，不作通过条件 | E1 |

**现有测试的变动**：删除 `Gripper.AnArmCommandThatStaysAwayAfterTheGripperIsAProtectiveStop`（由 A13 取代）；
`Gripper.NothingIsSentUnlessTheControllerStartedOnAUsableGripper`（武装）改写为 A7、A8；`Gripper.CommandsAreSentOnlyWhenTheyChange`
改写为 A10；`Gripper.AnArmMotionIsRefusedWhileTheGripperMovesWithoutDeactivating` 并入 A13；
`Gripper.TheOpeningIsHeldWhileTheFrameSaysZero` 扩展为 A11。

## Result

### 动代码前的三项真机预实验（2026-10-14 实测，FR5 + 大寰 PGI-140，控制器 v3.8.4.2）

在正式修改驱动代码前，由操作者在机械臂旁全程监护，直接通过 SDK / 状态帧高频轮询（与 E5 一致）完成夹爪实验，通过 `fr_servo_bench` 完成机械臂实验。

#### X1：`MoveGripper` 能否打断正在进行的夹爪运动

- **操作**：夹爪由 100 % 往 0 % 走，走到物理半程（约 1200 ms，开度约 73 %）时，由主机发出 `MoveGripper(1, 80, 50, 20, 30000)`。
- **实测数据与时间线**：
  ```text
  [    15.8 ms] Sent MoveGripper(0%) -> ok=1 code=0
      19.7 ms | pos=  0% | done=0  (G10 起始瞬态)
     909.5 ms | pos= 95% | done=0
  [  1200.4 ms] Calling MoveGripper(80%) (尝试打断)...
    1320.0 ms | pos= 73% | done=0
    1719.9 ms | pos= 49% | done=0
    2129.9 ms | pos= 26% | done=0
    2529.7 ms | pos=  7% | done=0
    2729.6 ms | pos=  0% | done=0  (原目标 0% 完整走完到底)
  [  2736.7 ms] MoveGripper(80%) returned after 1536.3 ms -> ok=1 err=0 code=0
    3640.0 ms | pos= 12% | done=0  (开始执行新目标 80%)
    4039.9 ms | pos= 34% | done=0
    4449.7 ms | pos= 57% | done=0
    4850.1 ms | pos= 77% | done=0
    5259.5 ms | pos= 80% | done=0
    5459.8 ms | pos= 80% | done=1  (新目标完成)
  ```
- **结论**：
  1. 第二次 `MoveGripper` **在控制器端被阻塞挂起整整 1536.3 ms**，直到前一个运动完全走完到达 0% 之后才返回 `code=0`。
  2. 夹爪**绝不立即转向**，而是**先走完原目标，再执行新目标**。
  3. `motion_done` 序列：前一个运动期间为 0；在原运动结束时因新运动紧接着开始，`motion_done` 保持为 0，直到 80% 到位后变为 1。
  4. **影响**：完全证实“控制器无法中止已发出的夹爪运动”。A5、A6 设计（取代和取消时必须等待当前运动走完）完全成立，计划无需修改。结论记入 `sdk_quirks.md` G15。

#### X2：夹住之后的帧序列，以及 `motion_done = 3` 何时出现

- **操作**：
  1. 发出 `MoveGripper(0%)` 夹住放置的物体，稳定报 `motion_done = 2`（位置 27 %）。
  2. 在夹住后 6 秒用力将物体抽出，状态帧 1 ms 高频记录抽走脱出瞬间及之后的变化。
  3. 从夹持状态（`done = 2`）直接发出 `MoveGripper(100%)` 张开指令，记录完整序列。
- **实测数据与时间线**：
  - **抽走脱落时间线**：
    ```text
    2319.2 ms | pos= 27% | done=2 | fault=0  (夹住物体，进入稳定夹持)
    8589.3 ms | pos= 23% | done=2 | fault=0  (操作者用力拉动物体，开度微变)
    8789.4 ms | pos= 23% | done=0 | fault=0  (物体脱出瞬间，阻力消失，夹爪继续闭合，done 变 0)
    8989.1 ms | pos=  6% | done=0 | fault=0  (高速闭合中)
    9199.0 ms | pos=  6% | done=3 | fault=0  (闭合到底未受阻，触发脱落，瞬间报 3)
    9399.0 ms | pos=  0% | done=3 | fault=0  (完全闭合到 0%，持续保持报 3)
    ```
  - **从夹住状态张开时间线**：
    ```text
        14.7 ms | ------------------------ EVENT: CMD MoveGripper(100%)
        14.7 ms | pos=  0% | done=0  (G10 瞬态，实际从 28% 张开)
       907.7 ms | pos= 32% | done=0
      1308.0 ms | pos= 54% | done=0
      1718.0 ms | pos= 78% | done=0
      2117.8 ms | pos= 97% | done=0
      2527.7 ms | pos=100% | done=0
      2727.8 ms | pos=100% | done=1
      Sequence of done: 2 -> 0 -> 1
    ```
- **结论**：
  1. 夹持命令（0%）在夹住后若物体脱出，控制器力矩闭环仍生效并驱动夹爪继续闭合，闭合到底未受阻后**报 `motion_done = 3` 且持续保持**！从脱出到报 3 仅约 400 ms。
  2. 从夹住状态（`done = 2`）发张开指令，序列严格为 **`2 -> 0 -> 1`**。“先报 0、再报非 0”的完成判定完全成立。
  3. **影响**：真机原样转发大寰的寄存器状态 3（掉落）。**计划中关于“报 3 → ABORTED / `gripper_object = dropped`”的处理完全保留**，A3 用例真实有效。

#### X3：180°/s 下滞后测量与 118 ms 组成拆解

- **操作**：J6 升余弦往返（从起点摆到 +摆幅再回来），周期 0.5 s，历时 4 s（8 个完整周期），分别在峰值 90、135、180°/s 下由 `fr_servo_bench` 记录每条命令点与状态反馈帧，通过互相关分析计算滞后时间与最大跟随误差；并进行 `prefill = 1` 的对照实验。
- **实测数据对比**（`cmdT = 8 ms`）：
  | 峰值速度 | 摆幅 | prefill | 互相关滞后 (lag) | 最大命令/反馈误差 | 固件队列均值 (`mc_queue_len`) | 结果 |
  | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
  | **90°/s** | 0 → 14.32° | 3 | **133 ms** | $10.48^\circ$ ($0.183\text{ rad}$) | 2.46 条 | 零故障，平滑完成 |
  | **135°/s** | 0 → 21.49° | 3 | **121 ms** | $14.81^\circ$ ($0.258\text{ rad}$) | 1.93 条 | 零故障，平滑完成 |
  | **180°/s** | 0 → 28.65° | 3 | **122 ms** | $19.57^\circ$ ($0.342\text{ rad}$) | 2.10 条 | 零故障，平滑完成 |
  | **135°/s (对照)** | 0 → 21.49° | 1 | **88 ms** | $11.29^\circ$ ($0.197\text{ rad}$) | 0.21 条 | 零故障，滞后骤降 33 ms |
- **结论**：
  1. 滞后不随速度发散：90、135、180°/s 下为 133、121、122 ms，连同 E1（≤ 90°/s，116–121 ms）在 116–133 ms 之间。
     最大误差与纯延迟模型相符：摆幅 A、周期 T、滞后 τ 时误差幅值为 A·sin(πτ/T)，180°/s 时 28.65°·sin(π·0.122/0.5) ≈ 19.9°，
     实测 19.57°。
  2. 180°/s 下最大跟随误差 0.342 rad。路径容差按滞后最大值 133 ms 取 0.85 rad（Approach 第 4 步），裕量约 2.5 倍；
     最快关节（3.2 rad/s）的预期误差 0.43 rad，裕量 2 倍。
  3. 滞后组成：实测的只有 `prefill` 对照——3 → 1 时滞后少 33 ms（121 → 88 ms），但队列均值只少约 1.7 条（1.93 → 0.21，
     约 14 ms），差值未解释。其余各项（主机端 `kLead` 约 24 ms、XML-RPC 往返约 4 ms〔S17 实测 3.8 ms〕、控制器内部插补与
     总线传输、伺服驱动器与机械响应、状态帧采样与传输）均为推断，未单独测量。
  4. `prefill = 1` 时队列均值 0.21，运动中大部分时间为空（欠载），与 S17（运动中队列须非空且稳定）冲突：降低 `prefill`
     不能直接用来减少滞后，留给减少滞后的专门计划。

#### X4：上电后未调 `GetGripperConfig` 时，状态帧的夹爪字段是否更新

- **操作**（2026-10-14，控制器重启后，示教器与 Web 端的夹爪页面均未打开；操作者在急停旁，机械臂不动）：
  1. 只读状态帧 5 s（不发任何 XML-RPC）；
  2. `ActGripper(1, 1)`；
  3. `MoveGripper` 50 %、100 %；
  4. 调一次 `GetGripperConfig`（只读），再重复第 3 步作对照。
- **实测**：
  ```text
  1. 500 帧：active=0x0000 pos=0% done=0 fault=0 temp=0 voltage=0
  2. ActGripper 21.8 ms 返回 0；739 ms 后 active=0x0001，pos=0% done=0（同 G5）
  3. MoveGripper(50)：0% → 95 / 73 / 52 / 50 %（0.82 / 1.23 / 1.63 / 2.03 s），2.23 s done=1
     MoveGripper(100)：0% → 54 / 76 / 97 / 100 %，2.29 s done=1
  4. GetGripperConfig → [0, 1, 3, 0, 0]（错误码 0，company=1，device=3）
     重复第 3 步：序列与时间同第 3 步（差 ≤ 40 ms）
  ```
- **结论**：
  1. 不调 `GetGripperConfig`，激活位、开度、完成标志都正常更新；调用前后行为一致。驱动不需要调用它。
  2. `GetGripperConfig` 读回 company 1（SDK 文档：1-Robotiq；大寰为 4）、device 3（文档未定义），但大寰 PGI-140
     工作正常、无故障：读回的厂商号与夹爪能否工作无关。
  3. `gripper_temp`、`gripper_voltage` 在 PGI-140 上始终为 0。
  4. 记入 `sdk_quirks.md` G16。

---

### 驱动代码修改与测试进度

Approach 第 1–6 步完成，每步一个提交、各自通过 `scripts/check.sh`；真机 D 组取消。最后一次 check：193 tests,
0 failures, 4 skipped，连续两次。

- **第 4 步**（速度缩放）：xacro 的 `<gpio name="${tf_prefix}speed_scaling">` 真机与 mock 共用；插件在 `read()` 里写因子，
  夹爪忙（运动或激活）且硬件激活时 0.0。mock 管线：JTC active、`dynamic_joint_states` 报 1.0、J6 轨迹 SUCCEEDED
- **扣臂删除**：`arm_held_`、2 s 超时、`ProtectiveStop::HoldTimeout` 删除；夹爪完成的周期起命令放行，只经步长检查。
  `~/status` 的 `arm_commands` 由硬件状态与 `gripper.busy` 推出，停用后不再残留
- **第 1 步**（插件提供 action）：相对名创建，测试客户端按 `/<ns>/gripper_controller/gripper_cmd` 连上（A15）。
  挂起目标的作废用一个计数（`gripper_epoch_`）：硬件激活/停用/出错、夹爪变为不可用、激活夹爪时加一，目标记下接受时的值，
  实时侧不发值不符的目标、定时器（20 ms）把它 ABORTED。实时侧对目标只 `try_lock`，运动结束经一个原子量
  （目标号 << 3 | 结果）交给定时器。mock 仍加载上游控制器，action 同名同类型、开合 SUCCEEDED
- **第 3 步**（原地运动开度）：第一版只要帧报过非 0 就采信 0 %，check 中偶发失败——发出后、控制器接下命令前，帧仍报原开度，
  提前解除了保持。改为运动开始（帧报过 0）后的非 0 才算；测试让假控制器延迟 100 ms 应答，确定性地覆盖这一窗口
- **第 5 步**：路径容差 0.85 rad
- **测试有效性**：每项新行为都把实现改回旧逻辑或关掉，确认新测试失败（因子恒 1、旧扣臂、无作废计数、只认 1 为完成、
  原地开度旧条件），改回后通过
- **与测试计划的出入**：A9（1.5 s / 30 s 超时）按维护者要求不做；A3 中"运动中报 3 → ABORTED"由测试在运动中改帧覆盖；
  B 组为 `test_trajectory_pause`（ControllerManager 循环 + JTC + 安装的 `controllers.yaml` + 插件 + 假控制器）：B1/B2 合为
  一个用例（+2°/0.5 s 与 +17°/0.5 s 线性），B5、B6 各一。B 组的有效性：插件因子恒 1 时 B1 以"command refused …
  2.000 deg after 0.000 deg"保护停止
- **顺带修复**：`on_configure` 清空 `frame_` 后，第一次 `read()` 若恰逢接收器加锁（`tryLatest` 失败），会把空帧判为过期、
  报 COMM（check 中一次 `no state frame in time`）；现在没有帧时等一次拷贝。偶发的 `ServoSender`、`StatusTopic` 墙钟测试失败
  记入 TODO

### 状态位实验与完成判定修正（2026-10-15）

控制器重启后（夹爪未激活，先由操作者 `ActGripper`），会话工具按 1 ms 轮询记录帧、经同一进程下发夹爪指令（日志在会话
机器 `/tmp/session/gexp.log`，不入库）。

| 试验 | 指令 | 过程与 `motion_done` |
|---|---|---|
| A | `MoveGripper(10 %, max_time 10 s)`，障碍物约在 28 % | 2.3 s 夹住报 2；15.9 s 抽走：2 → 0 → 3，停在 10 % |
| 张开 | 从 3（10 %）`MoveGripper(100 %)` | 0 ms 报 0；0.70 s 报 3、0 %；0.90 s 报 3、14 %；1.10 s 报 0；2.74 s 报 1 |
| B″ | `MoveGripper(10 %, max_time 3 s)` | 2.3 s 夹住报 2；约 100 s 后抽走：夹爪不动，一直报 2 |

**维护者结论**：`motion_done` 是一次运动内锁存的两个标志位，0x02 受阻（卡住）、0x01 到达；只在运动时限内更新
（sdk_quirks G13）。时限之后物体是否掉落不可观测。

**由此的改动**：

- **完成判定（缺陷）**：新运动开始后帧里会再现上一次的标志位（G17），"先报 0、再报非 0"会把它当成结束——误报结果，
  并在夹爪仍在动时放开机械臂（G9）。改为：报过 0，且标志位非 0、标志位与位置至少 300 ms 未变，且目标不为 0 % 时位置
  不为 0 %。代价：每次夹爪动作的结束晚约 300 ms 判定
- **`~/status` 删除 `gripper_object`**：时限后 2 不代表仍夹着、掉落不可观测，`held`/`dropped` 会误导；原始的
  `gripper_motion_done` 保留，README 写明其含义与局限
- 结果映射不变：1 SUCCEEDED（`reached_goal`），2 SUCCEEDED（`stalled`），3 ABORTED（运动中失去物体）

**结束（2026-10-15）**：维护者取消 D 组（D1–D7）——夹爪与机械臂命令本就不应同时发，不在真机上测并发；轨迹暂停作为
误用时的兜底保留，由 A、B 组覆盖。D6/D7（路径容差 0.85 rad 在高速下不误报、滞后组成）未做，容差依据仍是实测滞后
116–133 ms（S19）。最后一次 check：194 tests, 0 failures, 4 skipped。

