# 夹爪（大寰 PGI-140）接入 ros2_control

- Status: done（含修订 1；"无异响"一项转入 TODO.md）

## Goal

上层用标准接口开合夹爪：夹爪作为一个关节进入 ros2_control，由 `parallel_gripper_action_controller`
提供 `control_msgs/action/ParallelGripperCommand`，开度出现在 `joint_states`（后续 MoveIt 直接可用）。
激活只由操作者通过服务显式发起，驱动绝不自动激活（激活必然张开到满行程，sdk_quirks G2）。

## Done when

- 单测（假控制器）：命令变化才发 `MoveGripper`、参数正确（编号、开度%、速度%、力%、`block=1`）；
  未激活、故障、硬件非 active 时不发；激活服务在已激活时拒绝；`read()`/`write()` 内不做 XML-RPC
- mock 启动（`use_mock_hardware:=true gripper:=pgi140`）：RViz 里夹爪挂在 `tool0` 上、手指随动作开合
- 真机：
  - 操作者调用激活服务 → 张开、`~/status` 显示已激活；再调一次被拒绝
  - 机械臂静止保持（驱动在发伺服流）时 action 开 → 50% → 全开，`SUCCEEDED`，终点开度误差 ≤ 2%
  - 期间机械臂位形偏差 ≤ 0.05°、伺服队列不空、无故障
- `fr_robot_driver/README.md` 与 `gripper_dahuan_pgi140` 写明用法与安全约定

## Context

- 真机实测（`docs/sdk_quirks.md` G1–G7）：
  - `ActGripper(1, 1)` 约 20 ms 返回 0，0.9 s 内 `gripper_active` bit0 置位，**实际全开但状态帧报 0%**，
    直到第一次 `MoveGripper` 后位置才可信（G5）
  - `MoveGripper(1, pos, vel, force, max_time, block=1, 0, 0, 0, 0)` 非阻塞，约 40 ms 返回；
    **100% = 全开、0% = 闭合**；vel 20% 时约 22 %/s；到位后 `gripper_motiondone=1`（G6）
  - 激活状态属于控制器，每次上电只需激活一次（G1）；未激活时动作指令只会失败（G4）
  - `GetGripperConfig` 返回的编号与 SDK 头文件不符，但夹爪正常——只读不写（G7）
- 状态帧已有 `gripper_active`、`gripper_position`（%）、`gripper_motiondone`、`gripper_fault`、
  `gripper_current`、`gripper_speed`（`robot_state_384.hpp`）
- URDF（`gripper_dahuan_pgi140/urdf/gripper_dahuan_pgi140.urdf`）：根 `gripper_base_link`，
  `left_finger_joint` 0（闭合）–0.04 m（全开）、`right_finger_joint` mimic；目前是独立文件、无前缀、未挂到臂上
- 控制器：Jazzy 已装 `parallel_gripper_action_controller/GripperActionController`——命令接口 `position`，
  状态接口 `position`、`velocity`，可选命令接口 `max_velocity`、`max_effort`；MoveIt 有对应的
  `ParallelGripperCommand` 控制器句柄
- 驱动约定：故障期间不发运动指令、绝不自动复位；`ServoSender` 有自己的 20003 连接；插件节点 `<name>_hw`
  上已有 `reset_error`、`~/status`

## Approach

1. **描述**：`gripper_dahuan_pgi140` 改为 xacro 宏（带 `tf_prefix`，`parent` 默认 `tool0`，安装位姿作参数，
   默认 `xyz="0 0 0.0242" rpy="0 0 0"`，推导见 Questions 1）；
   `fr_controlled.urdf.xacro` 增加参数 `gripper`（`none` 默认 / `pgi140`），选中时挂到 `tool0`，
   并在 `fr_ros2_control.xacro` 的 system 里加入关节 `<prefix>left_finger_joint`
   （命令 `position`、`max_velocity`、`max_effort`；状态 `position`、`velocity`）。mock 分支同样加入
2. **客户端**：`RobotApi` 增加 `actGripper(index)`（仅激活，`action=1`）、`moveGripper(...)`、`getGripperConfig()`
3. **插件**：
   - `read()`：开度 = `gripper_position%/100 × 0.04` m；速度由相邻帧差分
   - **激活后、首次运动前**（状态帧报 0%，实际全开，G5）：不发指令纠正，而在驱动侧推断——
     `gripper_active` 已置位、`gripper_motiondone == 0`、`gripper_position == 0`，且驱动自看到该状态以来
     未发过 `MoveGripper` 时，开度报 **0.04 m（全开）**、速度 0；`~/status` 增加 `gripper_position_source`
     （`frame` / `inferred_open_after_activation`）。驱动发出第一条 `MoveGripper` 或任一条件不成立即回到状态帧值。
     依据：激活必然全开（G2），激活后 `motiondone` 保持 0，运动到位后为 1（G5、G6）
   - `write()`：命令开度与上次发出的相差 > 1% 时，把（开度%、速度%、力%）写入"最新值"槽，**不阻塞**；
     后台线程用独立的 20003 连接调 `MoveGripper(..., block=1)`，只发最新值
   - 不发的情形：未激活（`gripper_active` bit 未置位）、任何故障、硬件非 active
   - 换算：`max_velocity`（m/s）→ vel%（按实测 vel–速度关系标定，先线性），`max_effort`（N）→ force%（满量程 140 N）；
     action 未给（0）时默认 **force 20%、vel 50%**（常夹玻璃器皿）；编号参数 `gripper_index`，默认 1
4. **激活服务** `~/gripper/activate`（`std_srvs/Trigger`）：已激活 → 拒绝；未激活 → `ActGripper(index, 1)`，
   等 `gripper_active` 置位（超时 3 s）后返回；故障或机械臂运动中拒绝。驱动启动、configure、恢复时都**不**调用
5. **控制器**：`controllers.yaml` 增加 `gripper_controller`，只在 `gripper:=pgi140` 时 spawn；`goal_tolerance` 0.001 m（≈ 2.5%）

**备选（不采用）**：动作服务直接挂在 `<name>_hw` 节点（简单，但不进 `joint_states`、MoveIt 要另接）——维护者选了 ros2_control 方案。

**风险**：
- 推断规则依赖"激活后 `motiondone` 一直为 0、运动到位后一直为 1"——只观察过一次各十余秒；须真机验证
  （断电重启 → 操作者激活 → 驱动报全开；运动到 0% 后不误判为全开）
- `MoveGripper` 是否进入与 `ServoJ` 相同的 255 条运动队列、是否打断伺服——**未知**，真机验证项；若会，需改为只在机械臂静止时允许
- 夹持物体时开度停在中间、`motiondone` 何时置位未知——影响 action 的完成/stall 判定（`allow_stalling`）

## Questions

（已答）

1. **安装**：维护者：夹爪装在法兰平面上、随 J6 转；操作方向 = 法兰法线；J6 = 0 时手指运动平面与 J5 旋转平面平行。
   推导：`tool0` 的 z = 法兰法线（J6 轴），J6 = 0 时 J5 轴在 `tool0` 中为 −y，所以 J5 旋转平面 = x–z 平面，
   手指沿 `tool0` x 运动。夹爪模型中手指沿 `gripper_base_link` +z 伸出、沿 x 运动，与 `tool0` 约定一致 →
   **无旋转**；底座网格最低点 z = −0.0242 m → 偏移 +0.0242 m。线缆接头朝向（`rpy` 0 或绕 z 转 π）在 RViz 里与实物对照
2. **激活后开度**：不为此发指令，在驱动侧推断后修正 `joint_states`（Approach 3）
3. 默认 force 20%、vel 50%
4. 编号参数 `gripper_index`，默认 1

## 实施指引

**现状代码**（实施前先读）：

- 描述：`fr_robot_driver/urdf/fr_controlled.urdf.xacro`（`xacro:arg` 列表、调用 `fr_robot` 与 `fr_ros2_control` 宏）；
  `fr_robot_driver/urdf/fr_ros2_control.xacro`（`_fr_hw_joint` 宏、mock 分支 `mock_components/GenericSystem`、
  `<gpio name="<prefix>fr_status">`）；臂的 `flange` / `tool0` 在 `fr_description/urdf/fr_macro.xacro`。
  `gripper_dahuan_pgi140/CMakeLists.txt` 只安装 `collision`、`urdf` 目录
- 启动：`fr_robot_driver/launch/fr_control.launch.py`（`DeclareLaunchArgument` 列表、spawner）；
  控制器：`fr_robot_driver/config/controllers.yaml`
- 插件：`fr_robot_driver/src/fr_robot_hardware_interface.cpp`（`on_init` 解析关节与参数、创建 `<name>_hw` 节点与服务；
  `read()` 从 `StateReceiver` 快照取状态；`write()` 故障判定 → 步长检查 → `ServoSender::enqueue`）。
  `on_init` 目前按 6 个臂关节校验 `info.joints`，加入夹爪关节需放宽
- 状态话题：`fr_robot_driver/include/fr_robot_driver/status_report.hpp`（`StatusTracker`，`values` 里加夹爪字段）
- 客户端：`fr_client_library/src/robot_api.cpp`（XML-RPC 封装；参数形状对照 `references/libfairino`
  标签 `v2.2.4-robot3.8.4` 的 `robot.cpp`：`ActGripper(index, act)`、`MoveGripper` 10 个参数、
  第 8 个 `rotNum` 是 double；`GetGripperConfig` 无参数——**请求里不带 `<params>`**）
- 测试：`fr_client_library/test/test_robot_api.cpp`；`fr_robot_driver/test/fake_controller.hpp`
  （`serveXmlRpc` 按 `methodName` 分派，需加 `ActGripper` / `MoveGripper` 与状态帧夹爪字段）、
  `test_hardware_interface.cpp`；mock 描述的 xacro 检查见 `scripts/check.sh`

**真机验证**（规则见 AGENTS.md 规则 5"真机操作"）：

- 激活只由维护者授权；激活前确认手指之间无物、张开路径无障碍。夹爪的激活状态每次上电清零（G1），
  会话开始时用 `fr_probe` 读 `gripper: active=… position=… motion_done=…`，不要假设
- 推断规则的验证需要一次控制箱断电重启（由维护者操作），之后经激活服务激活
- 夹爪运动与伺服流的相互影响：驱动保持（伺服流在发、机械臂静止）时开合，看 `servo_queue_len` 与位形偏差
- 测单个 XML-RPC 时可用 `curl` 直接 POST 到 `http://<ip>:20003/RPC2`（须持 `flock /tmp/fr_robot_<ip>.lock`、驱动未运行）

## 修订 1（真机实测后，已批准）

原方案"机械臂静止保持（驱动在发伺服流）时开合夹爪"在真机上不成立（`docs/sdk_quirks.md` G8–G10、S18）：

- 伺服流进行中 `MoveGripper` 既不执行也不回复，直到流停下（队列排空即可，不必 `ServoMoveEnd`）；驱动 1 s 超时放弃的调用在驱动退出后照样执行了
- 夹爪运动期间 `ServoJ` 照收不执行：队列涨到 253，之后每条命令延迟约 2 s
- 每次夹爪运动开始约 0.8 s 内状态帧报 0 %，之后约每 0.4 s 更新一次
- 机械臂静止时停发 5 s 再恢复（先预填原地段）：偏差 ≤ 0.002°、无故障。维护者更正：S17 的顾虑是**运动中**的欠载（加速度变化、声响），不是刹车

另外，描述的手指方向原来是反的（0 = 全开）：已改为 0 = 闭合、0.04 m = 全开（轴向内、原点移到全开端），待在 RViz 里复核。

**修订后的做法**（替换 Approach 3 的 `write()` 与"不发的情形"，其余不变）：

1. **空闲停发**（`ServoSender`）：要发的点与上一条发出的逐字节相同（静止时命令每帧不变），且状态帧回读的各轴已到达该点，即停发，不调 `ServoMoveEnd`；
   之后第一条不同的命令先补 `prefill` 段原地不动的（上一目标 = 实际位置），再按原节拍续发。停发期间时钟偏差 PI 冻结。
   代价：每次从静止起步多 `prefill × cmdT`（24 ms）延迟。`~/status` 增加 `servo_stream`（`streaming` / `paused`），`fr_status/servo_queue_len` 停发时为 0
2. **夹爪命令只在停发时发**：`write()` 只有在伺服流已停发且状态帧队列为 0 时才把夹爪命令交给后台线程；机械臂在动时夹爪命令等着（只保留最新值）
3. **夹爪运动期间拒绝机械臂运动**（安全网；正常情况下上层等夹爪 action 结束再发轨迹，不会触发）：从交出 `MoveGripper`（或激活）起到完成为止（状态帧见过 `motion_done = 0` 之后变 1；激活为激活位置位；超时 30 s 报错。阻塞模式 `block = 0` 不可用，见 G11），
   与停住位置相差的机械臂命令不执行（伺服流保持停发），**不停用硬件**；`~/status` 报 `arm_commands: refused (gripper moving)`，日志限频告警。
   夹爪完成后，命令回到停住位置 0.01° 以内才恢复（不会跳过去；真机上按单周期步长限值放行时，J6 在一个周期里跳了约 1.3°，见 Result）。
   为让上层知道被拒，`joint_trajectory_controller` 加目标容差（每关节 `goal` 0.005 rad、`goal_time` 0.5 s）：被拒的轨迹 ABORTED，之后它保持在实际位置。
   夹爪与机械臂的先后由上层负责（等 action 结果再发下一步）
4. **开度**（G10，外设通信速率低）：夹爪运动中（`motion_done = 0`）帧报 0 % 时沿用上一个已知开度；激活后未运动过仍报全开（原推断规则是它的特例）。速度 = 两次开度变化之差 ÷ 间隔，0.6 s 无变化即 0
5. **激活服务**另要求伺服流已停发（或硬件未 active）；激活期间同第 3 条拒绝机械臂运动

**Done when 的修改**（替换真机第 2、3 条）：

- 机械臂静止 → 伺服流停发（`servo_stream = paused`、队列 0）；之后发一条小轨迹（单关节 ≤ 2°）：起步无故障、无异响、跟踪正常
- 夹爪 开 → 50 % → 全开，各 `SUCCEEDED`，终点开度误差 ≤ 2%；期间机械臂偏差 ≤ 0.05°、无故障
- 夹爪运动中发轨迹：被拒（ABORTED），机械臂不动、硬件仍 active；夹爪完成后同一轨迹可执行

## Result

实现：描述（`gripper_dahuan_pgi140` 宏挂到 `tool0`，`gripper:=pgi140`）、`RobotApi` 的 `actGripper`/`moveGripper`/`getGripperConfig`、
`GripperSender`（独立 20003 连接、只发最新值）、插件的夹爪关节 / 武装 / 激活服务 / `~/status` 字段、
`ServoSender` 的空闲停发、夹爪与机械臂轮流动、`gripper_controller` 与轨迹控制器目标容差。`scripts/check.sh`：180 个测试通过
（夹爪 8 个、停发 1 个；假控制器模拟队列、G10 的 0 %、G11 的激活清故障，并统计"队列未空时收到 `MoveGripper`"
与"夹爪运动中收到 `ServoJ`"，测试要求两者为 0）。

真机（3.8.4.2，FR5，夹爪编号 1，J6 允许 ±30°）：

| 项 | 结果 |
|---|---|
| mock + RViz | 夹爪挂在 `tool0`，手指随 action 开合（维护者目视） |
| 静止停发 | 启动后 `servo_stream = paused`、队列 0；J6 +2°/4 s 与 +10°/5 s 往返：`SUCCEEDED`，终点误差 ≤ 0.0022°，运动中队列 2–4，停下后重新停发，无故障 |
| 夹爪 50 % → 全开 | 各 `SUCCEEDED`，终点 50 % / 100 %（误差 0）；机械臂偏差 ≤ 0.0028°，`MoveGripper` 0 失败；运动中帧报 0 % 时开度保持（`held_during_motion`） |
| 夹爪运动中发 J6 +2° 轨迹 | 被拒，J6 不动，硬件保持 active，轨迹 ABORTED（goal_time 容差）；之后同一轨迹 `SUCCEEDED` |
| 再次激活 | 已激活时被拒，未发 `ActGripper` |
| 断电重启后激活 | 重启后 `gripper_active = 0`（G1）；`gripper_controller` 未武装；`~/gripper/activate` 成功，开度报全开（`inferred_open_after_activation`），再调被拒；重启 `gripper_controller` 后武装；合到 0 % 报 0（`frame`，未误判为全开），再开到 100 % |
| RViz（修正后的手指方向） | 正常（维护者目视） |

过程中的发现（已写入 `docs/sdk_quirks.md` G8–G11、S17 更正、S18）：伺服流中 `MoveGripper` 不执行也不回复（原方案不可行，见修订 1）；
夹爪运动中 `ServoJ` 照收不执行；运动开始时帧报 0 %；`block = 0` 使夹爪失去激活并报故障，`ActGripper(1, 1)` 即可恢复；
描述的手指方向原来是反的。第一版"夹爪完成后命令回到停住位置一步以内即恢复"在真机上让 J6 在一个周期里跳了约 1.3°
（单周期步长限值即 URDF 最高速度），改为 0.01° 后被拒的轨迹 ABORTED、不再跳。

未激活时帧报 0 %，开度实际不可知（重启前全开，帧报 0）；收尾后改为报全开（外轮廓最大，碰撞检查偏保守，
`gripper_position_source = unknown_not_active`，`da20127`），README 已写明。
异响只能用更大的运动来判断，本次轨迹太小，留作后续（`TODO.md`）。
