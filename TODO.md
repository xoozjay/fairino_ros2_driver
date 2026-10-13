# TODO

未决事项、未验证的断言与已知风险。设计决策与现状见 [AGENTS.md](AGENTS.md)、各包 README 和
`docs/`；改动历史见 `git log`。

## 待办

### 驱动（真机）

- [ ] `comm_timeout_periods` 取值 —— done when: 有系统的真机实测依据（默认 3 个周期 = 24 ms，真机约 1 h 使用中未误报）
- [ ] FK 一致性检查在真机上的实际偏差与容差（位置 / 姿态）—— done when: 真机多姿态实测记录
      （正装真机起始位形：0.101 mm / 0.0005°，`flange_cur_pos` 单位 mm/deg）
- [ ] 非正装（倾斜、倒装）时的基坐标系约定 —— done when: 真机确认 `flange_cur_pos` 的参考系
- [ ] 型号表除 FR5 V6.0 外各行的真实型号字符串 —— done when: 实物或厂商确认
- [ ] 伺服时钟偏差补偿的 trim（`ServoStats::trim_ppm`）不可观测，撞 ±1 % 限幅时只表现为队列漂移 —— done when:
      决定是否加到 `~/status`
- [ ] 状态帧新鲜度只看接收时间，不查 `frame_cnt` 递增：控制器发合法但帧号冻结的帧时漏判（风险极低）
      —— done when: 决定是否在 `read()` 里加帧号检查
- [ ] 真机双机（`dual_fr_cell.launch.py`，显式 `alice_ip` / `bob_ip`）—— done when: 两台真机跑通
- [ ] 整条轨迹交给控制器执行（对标 UR 的 `passthrough_trajectory_controller`；3.8.4 候选 `NewSpline*`、`LoadTrajectoryJ`/`MoveTrajectoryJ`）—— done when: 评估精度、延迟与中止语义后决定是否立计划

### 规划层（fr_moveit_config）

- [ ] 对 `fr_description` 跑 MoveIt Setup Assistant（tip 选 `flange`），替代
      `references/fairino_vendor/` 里的 7 份生成物
- [ ] SRDF / kinematics / controllers 对接新命名（语义关节名）
- [ ] 多机规划场景（双臂互碰检查）——ros2_control 不查碰撞，工作台 demo 的互穿轨迹要靠它拒绝

### 单元组合层

- [ ] 空闲停发 / 恢复与夹爪轮流动时的异响 —— done when: 用较大、较快的机械臂运动实测（小幅轨迹听不出）
- [ ] 工作台场景升级为完整单元（对标 `my_robot_cell` / `my_dual_robot_cell`）
- [ ] 评估 Gazebo / Ignition 仿真路线（当前只有 mock）

### 杂项

- [ ] 本仓库尚未配置远程；首次 push 需 `git lfs push --all` 同步 mesh 对象（96 个 LFS 文件）
- [ ] `initial_positions.yaml`（单机默认位形）与工作台位形的关系可考虑写进
      [docs/extension_pattern.md](docs/extension_pattern.md)

## 未验证

以下行为只在 3.8.3 仿真器上观察过；**仿真器时序数据无效**，真机取值待确认：

- 状态推送周期最小值（仿真器 8 ms，≤ 7 ms 被拒绝并返回码 4）
- 8080 即时指令回执内容 `1` 的含义与失败取值；收发超时默认 1000 ms
- 参数个数不符时控制器的处理（`ERR_PARAM_NUM`）
- SI0/SI1 在急停以外的原因下能否由 `ResetAllError` 清除（急停后不能，见 `docs/sdk_quirks.md` F4）
- 3.7.6–3.8.3 与 3.8.4 之外的状态帧：只做过 SDK 各 tag 静态比对，未上真机

## 已知风险

- **`ServoJ` 往返最大 7.7 ms，接近 8 ms 周期**（真机 S17）：网络更差时发送会落后，`ServoSender` 追赶、
  固件队列吸收 —— impact: 队列抖动变大，`prefill` 余量被吃掉
- **急停在本硬件上表现为控制箱断电**（F4）：断线期间驱动只能报"链路中断"，分不清急停、掉电与网络，
  要等控制器重新上线、`~/status` 报 1/149 才能确定 —— impact: 断线那 ~20 s 内上层只知道"断了"
- **控制器 20004 与 8080 单客户端**：驱动运行时诊断工具（`fr_probe`、`fr_servo_bench`）连不上，
  排障期间只能停驱动 —— impact: 真机排障窗口
- **`ServoJ` 不受全局速度/加速度限制**（[docs/sdk_quirks.md](docs/sdk_quirks.md) S14）：
  保护只在 ROS 侧（URDF 限位、`enforce_command_limits`、单周期步长检查）—— impact: 配置错误会直接撞机
- **故障恢复全由操作者发起**，驱动绝不自动复位（[docs/fault_handling.md](docs/fault_handling.md)）
  —— impact: 无人值守时故障即停机
