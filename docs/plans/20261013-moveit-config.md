# MoveIt 配置（fr_moveit_config）与法兰整圆验收

- Status: draft（批准后提出修订，待维护者再批：夹爪两指的自碰撞禁用；夹爪 goal 的执行时长监控列为阻塞性前置条件。
  见 Done when 末两条、Approach 1、4）

## Goal

单臂（任一型号，可选夹爪）能用 MoveIt 规划并经现有驱动执行：自由运动（OMPL）、直线与圆弧（Pilz
LIN / CIRC）。以维护者给定的法兰整圆（TODO.md "验收：法兰走整圆"）作验收，先 mock、后真机。

## Done when

- 新包 `fr_moveit_config`：`ros2 launch fr_moveit_config move_group.launch.py model:=… [tf_prefix:=… namespace:=… gripper:=pgi140]`
  连到已在运行的 `fr_control.launch.py`（mock 或真机），RViz MotionPlanning 可规划、执行（维护者目视）
- `scripts/check.sh` 覆盖：每个型号的 SRDF 能与 URDF 一起被 MoveIt 加载（不起 move_group 的轻量检查）
- 整圆脚本 `fr_moveit_config/scripts/flange_circle.py`（Pilz 序列，两段 CIRC）：
  - `--dry-run`：只规划，打印时长、各关节最大速度，以及 P2 混合区内法兰线速度的最小值与路径偏离圆的最大距离
  - mock：执行成功；在 P1、P2、P3 与最左点处，由 `joint_states` 经 FK 得到的法兰位姿与目标差 ≤ 1 mm / 0.1°
    （P2 在混合区内，按实测偏离记录）；混合区外的匀速段线速度与设定值相差 ≤ 2%
  - 真机（维护者在场、批准速度）：先 10%、再 30% 各走一圈，终点与 P1 偏差 ≤ 1 mm（`fr_probe` 的 `flange_cur_pos`），
    无故障、伺服队列不空；记录各点法兰误差
- README（包内）写明用法与安全约定
- （修订）`gripper:=pgi140` 时 SRDF 禁用 `<prefix>left_finger_link`–`<prefix>right_finger_link`：两指网格在任何开度都互相穿插
  （闭合约 47 mm、全开约 12 mm，厂商模型亦然），二者是兄弟 link，默认碰撞矩阵不会禁用，不禁用则每次规划都报自碰撞
  （维护者已同意）。**夹爪与机械臂之间的碰撞须保留**：夹爪各 link 与非相邻的机械臂 link 不得因 `collisions_updater`
  采样"从未碰撞"而禁用（采样会漏掉少见位形），只禁用相邻对与两指之间；有测试位形让夹爪碰前臂时规划被拒
- （修订）夹爪 goal 经 MoveIt 执行前，执行时长监控已解决（见 Approach 4）；未解决前只用整圆等不含夹爪的验收

## Context

- 已有：`fr_description`（参数化宏，语义关节名 `shoulder_pan_joint` … `wrist_3_joint`，`flange` / `tool0`，
  支持 `tf_prefix`）；`fr_control.launch.py`（`model`、`tf_prefix`、`namespace`、`gripper`、`use_mock_hardware`）；
  控制器 `<ns>/joint_trajectory_controller`（`FollowJointTrajectory`，目标容差 0.005 rad、路径容差 0.26 rad）、
  `<ns>/gripper_controller`（`ParallelGripperCommand`）
- `references/fairino_vendor/fairino*_moveit2_config`：Setup Assistant 生成物，**旧关节名 `j1..j6`**、
  KDL、`joint_limits.yaml` 加速度一律 0.7 rad/s²、`pilz_cartesian_limits.yaml`（1.0 m/s、2.25 m/s²、1.57 rad/s）。
  只作参考，不复制
- 已装：MoveIt（Jazzy）、OMPL、Pilz（含 `MoveGroupSequenceAction`）、CHOMP、STOMP；
  `moveit_setup_assistant/collisions_updater`（命令行生成自碰撞禁用表）。**未装 `moveit_py`**——脚本用 rclpy 动作客户端
- URDF 关节限位只有位置与速度（FR5：J1–J3 3.15 rad/s），没有加速度；Pilz 与时间参数化需要加速度限
- 驱动侧保护：单周期步长 ≤ URDF 最高速度 × cmdT；伺服流空闲停发（`docs/sdk_quirks.md` S18）；
  机械臂与夹爪须轮流命令；误用时（夹爪忙时来了轨迹）轨迹经 `speed_scaling` 暂停、夹爪结束后继续
  （`fr_robot_driver/README.md` "夹爪"）
- 整圆（FK 已核对：给定关节与位姿相差 0.10 mm）：`base_link` 下 y = −400 mm 平面，圆心 (191.6667, −400, 430)，
  半径 108.3333 mm，姿态 (90, 0, 0) 不变；最左点 L = (83.3333, −400, 430)

## Approach

1. **手写配置**（不用 GUI 生成，以支持 `model` / `tf_prefix` 参数化）：
   - `srdf/fr.srdf.xacro`：组 `<prefix>arm`（chain `<prefix>base_link` → `<prefix>flange`）；`gripper:=pgi140` 时加组
     `<prefix>gripper`（`left_finger_joint`）与末端执行器；自碰撞禁用表按型号用 `collisions_updater` 生成一次后入库
     （`config/<model>/collisions.srdf.xacro`，可复现命令写进 README）
   - `config/kinematics.yaml`（KDL）、`config/joint_limits.yaml`（加速度见 Questions 2）、`config/pilz_cartesian_limits.yaml`、
     `config/ompl_planning.yaml`、`config/moveit_controllers.yaml`（`<ns>/joint_trajectory_controller`，可选夹爪控制器）
   - `launch/move_group.launch.py`：用 `MoveItConfigsBuilder` 组装，robot_description 由同一个
     `fr_controlled.urdf.xacro` 生成（与驱动一致）；默认规划管线 OMPL，加载 Pilz 与 `MoveGroupSequenceAction` 能力
2. **整圆脚本**（维护者：用两段 CIRC，不自写轨迹生成）：PTP 到 P1（关节目标，用给定关节值）→ Pilz 序列
   [CIRC P1→P2 经 P3，CIRC P2→P1 经 L]，两段在 P2 处混合（`blend_radius` 取参数，默认 0.02 m，不在 P2 停顿）。
   L 由三点所定的圆算出（不是新点），两段都在同一个圆上。若 Pilz 拒绝 CIRC–CIRC 混合，退回 `blend_radius = 0`
   （P2 处短暂停顿）并在 Result 记录。速度 / 加速度缩放取参数
3. **真机步骤**：维护者确认现场与急停 → 驱动启动 → 10% 移到 P1，停，确认 → 先 `--dry-run` → 10% 整圆 → 30% 整圆 →
   每步后 `fr_probe` 记录法兰位姿

4. **（修订）夹爪与 MoveIt 的执行监控（阻塞性前置条件，做法待批）**：MoveIt 执行每段轨迹时按"期望时长 × 1.1 + 0.5 s"
   （`trajectory_execution` 的默认值）计时，超时即取消 goal、执行报 `TIMED_OUT`。夹爪 goal 是单点、期望时长约 0，
   允许只有 0.5 s，而真机夹爪运动一般 2–5 s，最长 30 s（驱动对夹爪 goal 的上限）。现驱动由插件直接提供夹爪 action，
   已发出的运动被取消时照常走完、不会开回去，但 MoveIt 仍把一次正常的开合判为失败、中止后续执行。
   提议：只对夹爪控制器放宽，`moveit_simple_controller_manager.<ns>/gripper_controller.allowed_goal_duration_margin: 31.0`
   （MoveIt 2 按控制器读取该参数，覆盖全局值；机械臂仍用默认监控；带命名空间的控制器名能否作参数名须实测）。经维护者批准后再接入夹爪

**不在本计划**：双臂规划场景（另立计划，TODO 保留）；工作台碰撞物体；整条轨迹交给控制器执行。

## Questions

（已答：1 是，线速度恒定；2 用 vendor 值；3 两段 CIRC，在 P2 混合（曾考虑自写恒速轨迹，维护者否决：太复杂）；4 是，双臂另立。原问题保留如下。）

1. **"30%" 的含义**：我理解为 MoveIt 的 `max_velocity_scaling_factor = 0.3`。对 Pilz CIRC，它缩放笛卡尔限速——
   若沿用 vendor 的 1.0 m/s，即法兰线速度 0.3 m/s，一圈（0.68 m）约 2.5 s 加上加减速；对 PTP，它缩放关节限速（FR5 J1–J3
   180°/s × 0.3 = 54°/s）。是这个意思吗？首次真机先 10% 可以吗？
2. **加速度限**：手头没有实测或厂商数据。先用 vendor MoveIt 配置的值（关节 0.7 rad/s²，笛卡尔 2.25 m/s²）？
3. **整圆在 P2 处**：两段 CIRC 之间不混合（`blend_radius = 0`，在 P2 短暂停顿，最简单、可预期），还是要求一笔连续
   （混合半径 > 0，Pilz 对 CIRC–CIRC 混合有限制，需实测）？
4. 范围：本计划只做单臂 + 可选夹爪，双臂另立——可以吗？

## Result
