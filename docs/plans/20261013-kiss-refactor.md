# 合入前简化驱动：夹爪改 action、统一数据通路、改用现成库、整理提交

- Status: abandoned（范围过大，拆分为合入前计划 [20261013-pre-merge-cleanup.md](20261013-pre-merge-cleanup.md)
  与合入后的单项计划，见 Result）

## Goal

`agent/dev` 能用（真机验证过、`scripts/check.sh` 全绿），但复杂度偏高，且有几处并发与语义缺陷。
合入 `main` 之前按 KISS 收敛：删掉不用的代码与越权接口；修掉对象生命周期竞态；状态帧只留一条交接路径；
能用成熟库的地方用成熟库（XML-RPC 编解码与 HTTP、FK）；夹爪从"ros2_control 命令关节"改为 action，
与机械臂的互斥只在一处实现；最后把 `agent/dev` 的提交整理成可读、可 bisect 的序列再合入。
MoveIt 计划（[20261013-moveit-config.md](20261013-moveit-config.md)）依赖的接口
（`<ns>/gripper_controller/gripper_cmd`，`ParallelGripperCommand`）保持不变。

## Done when

- `scripts/check.sh` 通过；以下代码不再存在：`GripperSender`、`TripleBuffer`、`xmlrpc_codec`、`HttpConnection`、
  `kinematics.hpp`、`ServoSender::resume`、`perform_command_mode_switch`，以及 W2 列出的未用 API；
  URDF 不再传 `kinematics` / `flange_offset` 参数
- 新增常驻测试：多线程执行器下"status 定时器运行中重新 configure"（W1 的竞态回归）
- 单测覆盖新夹爪行为：move 成功（`reached_goal`）、夹住（`stalled`，成功）、已激活时拒绝激活、激活以 active 位为完成、
  机械臂未停稳时等待后执行、等待超时拒绝、cancel 被拒；夹爪忙时机械臂命令变化 → 扣住，夹爪结束后命令回到停住位置即放行，
  超时 → 保护停止（`~/status` 写明原因）
- 一次性 ThreadSanitizer 构建（独立的 build/install 目录）跑两包测试，本仓库代码无数据竞争报告
- devcontainer 镜像按新依赖 `--no-cache` 重建后 check 通过
- 真机（操作者发起夹爪指令，AGENTS.md 规则 5）：夹爪计划 Result 的表格重跑一遍；"JTC SUCCEEDED → 伺服流停发"实测时长
  记入本计划；空闲停发与恢复（S18）复测；FK 检查偏差与现实现一致（0.101 mm / 0.0005° 量级）；
  拔网线 → 重新 configure → activate 不崩溃
- 整理后的分支每个提交通过 check，维护者合入 `main`

## Context

审查（读最终代码，对照 ros2_control / ros2_controllers / MoveIt 的 Jazzy 源码与 xmlrpc-c 文档；本计划草稿另经一轮独立核对）
发现的问题，按工作项归组。"推演"指读代码得出、未实测。

**对象生命周期竞态（推演）**：`sender_`、`gripper_sender_` 是 `unique_ptr`，在 `on_configure` 里重新赋值
（`fr_robot_hardware_interface.cpp:407`、`:411`）；status 定时器在本插件节点的回调组里解引用它们
（`:640`、`:649`），激活服务也读 `sender_`（`:1063`）。`ros2_control_node` 用 `MultiThreadedExecutor`
（`references/ros2_control/controller_manager/src/ros2_control_node.cpp:41`）。`read()`/`write()` 与 lifecycle 已由
ResourceManager 的 `resources_lock_` 串行化（`resource_manager.cpp:2381`、`:2431`），竞态只存在于插件节点的回调与
lifecycle 之间 → 断线后重新 configure（文档写明的恢复流程）时可能访问已释放对象。`receiver_` 为此改过
`shared_ptr + receiver_mutex_`，另两个没有。现有测试用单线程执行器，覆盖不到这种并发。

**线程入口无兜底**：`servo_sender.cpp`、`gripper_sender.cpp`、`state_receiver.cpp` 的线程函数与插件的 cleanup /
`monitorLink` 线程都没有最外层 try 块；异常逃逸 → `std::terminate` → 整个 controller_manager 退出（双机时两台都停）。

**状态帧两条交接路径**：`StateReceiver` 每帧写 `TripleBuffer`（单消费者，供 `read()`）再写一份 mutex 副本
（供其他所有读者）。新鲜度判断写了 6 处、2 个阈值；"步长故障覆盖为 RobotFault"在 `read()` 与
`StatusTracker::update` 各写一遍。热路径上的读者只有两个：`read()`（控制器管理器实时线程）与 ServoSender 线程
（每 cmdT 调 `queue_depth`，静止时调 `at_rest`；可选 `SCHED_FIFO`）。其余读者（status 定时器 10 Hz、lifecycle 回调、
激活服务）不在热路径。

改为单个 mutex 后，热路径读者用 `try_lock`，拿不到就沿用上一帧。不会饿死：写者每帧（8 ms）持锁一次、只做约 1 KB
拷贝（约 100 ns）；`read()` 按主机时钟、写者按控制器时钟到达，两者相差约 1000 ppm（S17）且网络到达有数十 µs 抖动，
相位不会长期重合。即便连续失败，沿用的帧带着旧的 `received_at`，超过 `comm_timeout_`（默认 24 ms）就按 COMM 处理——
最坏情况是一次误报的通信故障，不会是用了错误的数据。ServoSender 两个回调对"拿不到"已有安全语义：`queue_depth`
返回 `nullopt` 保持 trim，`at_rest` 返回 `false` 推迟停发。

**未用与越权代码**：`RobotApi` 的 `robotEnable`、`setMode`、`setSpeed`、`dragTeachSwitch`、`setLoadWeight`、
`setLoadCoord`、`setToolCoord`、`setDO`、`setToolDO` 只有单测调用——其中使能、负载、工具正是 AGENTS.md 规则 5
规定驱动不得做的；`InstantCommand` 的 Pause/Resume/ExtAxisStopJog/ConveyorComDetectTrigger 从不发送；
`ServoSender::resume()` 驱动不调用（恢复走 `start()`）；`StateReceiver::error()` 只有单测用；
`on_configure` 里 `getGripperConfig` 只为打一行日志（G7：返回值本就与头文件编号不符）。

**XML-RPC 是自研的**：`xmlrpc_codec`（按 XmlRpc++ 0.7 的报文格式）+ `HttpConnection`（keep-alive 轮换、毫秒超时、
不重试），含测试约 900 行。控制器端是 xmlrpc-c 的 Abyss 服务（C7：`Server: Xmlrpc-c_Abyss/1.54.5`），同源的
xmlrpc-c 报文天然兼容。Ubuntu noble 提供 xmlrpc-c **1.33.14**（rosdep 键 `libxmlrpc-c++` → `libxmlrpc-c++8-dev`，
容器内尚未安装）。它的完整客户端（Curl 传输）不适合伺服流：超时向上取整到整秒；`setInterrupt` 是为信号设计的，
别的线程只置标志不会唤醒 `pselect`，每轮最多等 3 s（1.33.14 `xmlrpc_curl_transport.c`）。因此只用它的编解码
（`generateCall` / `parseResponse`，服务端侧 `parseCall` / `generateResponse` 供 fake controller），HTTP 交给
libcurl 的 easy 接口（`CURLOPT_TIMEOUT_MS`，同一 handle 复用即持久连接）。8080 与 20004 是私有二进制协议，
没有现成库，`TcpSocket` 保留。

**FK 是自研的**：`kinematics.hpp`（148 行）做法兰 FK，假定每个关节都绕自身 z 轴转（`kinematics.hpp:28`）；所需的
6 组关节原点与 `flange_offset` 由 xacro 从 YAML 拼成参数传入（`fr_robot_driver/urdf/fr_ros2_control.xacro:154`），
与 URDF 重复。`fairino3_mt_v6` 的关节轴是倾斜或取负的（`fr_description/config/fairino3_mt_v6/default_kinematics.yaml`，
如 `axis: 0.00011251 -0.034899 0.99939`），现实现对该型号算错（推演，无该型号实物）。`HardwareInfo::original_xml`
是完整 URDF（`component_parser.cpp:799`），`kdl_parser` 可直接建链并正确处理任意轴。

**`fr_status` gpio 与 `~/status` 重叠**：gpio 的 11 个状态接口与 `~/status` 字段大量重复；gpio 在硬件 unconfigured
时随 broadcaster 一起停发，`~/status` 不会。缩减 gpio 是接口变更（`docs/fault_handling.md` §7）。

**夹爪架构**：夹爪现在是 ros2_control 关节（命令 `position`/`max_velocity`/`max_effort`），由
`parallel_gripper_action_controller` 每周期写入；驱动在 `write()` 里用死区从电平还原"新命令"、等伺服流停发、
经 `GripperSender` 发出，并在实时线程里仲裁机械臂（`arm_held_`、0.01° 交还）。这套接法照搬的是 Robotiq 一类
"电平型、与机械臂并行"的夹爪（PickNik `ros2_robotiq_gripper`：位置寄存器可随时改、后台线程持续写入）。
PGI-140 经法奥控制器是另一类：离散命令、反馈慢且会错报（G1、G5、G10），并且与机械臂互斥——
同类先例是 Franka Hand：独立的 `move`/`grasp`/`homing` action 加给 MoveIt 用的 gripper action，状态另行发布。
MoveIt Jazzy 有 `parallel_gripper_command_controller_handle.hpp`，真机改由插件提供同名同类型 action 后，MoveIt 配置不必分支。

互斥是控制器的硬约束（实测）：

- 伺服流进行中 `MoveGripper` 不执行也不回复，流停下（队列排空）后才执行（G8）
- 夹爪运动期间 `ServoJ` 照收不执行，固件队列涨到约 253，夹爪完成后才消费，之后每条延迟约 2 s（G9）
- 固件队列满（约 255 条）后每次 `ServoJ` 调用阻塞到有空位（S16）

所以夹爪忙时机械臂物理上动不了，驱动只能决定"机械臂命令变化时怎么告诉上层"。现设计由此产生的问题：

- 完成由两处各自判断：驱动看 motiondone，控制器看开度进 1 mm 或停滞 2 s（`allow_stalling: false` → 夹住物体报
  ABORTED，`fr_robot_driver/config/controllers.yaml`）
- （推演）goal 到达时机械臂未停稳 → 2 s 后 ABORTED，但目标开度仍留在命令接口里，机械臂停下后驱动照发
  MoveGripper：夹爪在 action 失败之后才动
- （推演）cancel 时控制器 `set_hold_position()` 取"当前开度"（`parallel_gripper_action_controller_impl.hpp:201`），
  运动前 0.8 s 帧报 0 %、驱动沿用旧开度（G10）→ hold 值是旧开度 → 运动结束后驱动再发一条把夹爪开回去
- 审查发现 M0：夹爪运动中上层发了机械臂指令，臂可能被无限期扣住，只有一条 WARN
- 激活后须重启 `gripper_controller`（arming）

**激活与运动的完成条件不同**：激活后帧里 `motiondone = 0`、位置 0 % 可持续 12 s 以上（G5），完成只能看 active 位置位且
`gripper_fault == 0`（现 `onGripperActivate` 的做法）；运动则看 motiondone 0 → 1（G6、G10）。

**顺序执行的时序**：JTC 报 SUCCEEDED 的条件是位置进 goal 容差 0.005 rad（约 0.29°）且速度 < `stopped_velocity_tolerance`
0.01 rad/s（约 0.57°/s）；伺服流停发要求实际位置与指令差 ≤ 0.01°（`kAtRestDeg`）。所以 JTC 报成功时机械臂可能仍在收敛，
夹爪 goal 紧接着到达是常态，必须等待而不是拒绝（现设计是排队等待）。

**被扣住的轨迹何时结束**：`controllers.yaml` 没有配置路径容差（`trajectory`），JTC 默认 0 即不检查
（`joint_trajectory_controller/tolerances.hpp` 注释）。被扣住的轨迹会按时间跑到终点、再过 `goal_time` 0.5 s 才因 goal
容差 ABORTED，之后 JTC 保持实际位置，命令才回到停住位置。也就是说，夹爪结束后命令多久回来取决于被扣轨迹的剩余时长，
而不是"一个周期"。

**依赖与环境**：新增 `libxmlrpc-c++`、libcurl（`fr_client_library`）；`kdl_parser`、`orocos_kdl_vendor`、
`control_msgs`、`rclcpp_action`（`fr_robot_driver`）。每个都要进 `package.xml`；devcontainer 镜像的 rosdep 层按
`package.xml` 安装，加依赖后须 `--no-cache` 重建（`.devcontainer/Dockerfile:113`）。

**提交历史**：`main..agent/dev` 69 个提交（36 个 docs）；`ServoSender` 前后重写 4 次，中间版本已无意义；
两个测试执行器修复（`8b44832`、`a903b0b`）针对同一个问题。

## Approach

各工作项独立提交、每个提交通过 check。顺序：W2 → W1 → W3 → W5 → W4 → W6 → W6b → W7 → W8。
W2 先做，减少后续要改的代码；W1 不碰 `GripperSender`（W5 删除它）。

**W1 并发与生命周期**
- `StateReceiver`、`ServoSender` 在 `on_init` 创建一次，不再重新赋值；软件版本改为 `StateReceiver::start(version)`
  的参数，ServoSender 的回调选项在 `start()` 传入；`fr_probe`、`fr_servo_bench` 随签名一起改。随之去掉
  `receiver_mutex_` 与 `shared_ptr` 续命（`monitor_rx_` 同样只建一次）
- 每个线程入口（ServoSender、StateReceiver、cleanup、`monitorLink`）包一层 `catch (...)`：前两者转成与传输错误相同的
  状态（`comm_error_` / `running_ = false`），走已有的 COMM 路径；后两者记 ERROR 日志后结束
- 常驻测试：`MultiThreadedExecutor` 上 status 定时器运行时反复 configure / cleanup
- 一次性 TSan：`colcon build --build-base build_tsan --install-base install_tsan --cmake-args -DCMAKE_CXX_FLAGS=-fsanitize=thread`
  后跑两包测试；ROS/DDS 未插桩造成的误报用临时 suppression 文件（不入库）排除；只看本仓库代码的报告

**W2 删除未用与越权代码**：Context 所列 API、`InstantCommand` 四项、`ServoSender::resume` 及
`resume_mutex_`/`applyResume`、`StateReceiver::error()`、`getGripperConfig`（调用与 API），连同各自的单测与
fake controller 分支

**W3 状态帧单一路径**
- `StateReceiver` 只留 mutex 保护的最新帧：`latest(out)`（阻塞）与 `tryLatest(out)`（`try_lock`，拿不到返回 false）。
  `read()` 与 ServoSender 的两个回调用 `tryLatest`，其余用 `latest`；写者在锁内只做一次拷贝。删除 `TripleBuffer`
- 一个新鲜度辅助函数，一个"有效故障"函数（帧分类 + 驱动自设的保护停止），`read()` 与 `StatusTracker` 共用
- 不用 `realtime_tools::RealtimeThreadSafeBox`：做法相同，但 `fr_client_library` 不依赖 ROS

**W5 夹爪改 action**
- 真机分支下夹爪关节只有状态接口（`position`、`velocity`），JSB 照常发布，碰撞检查与 RViz 不受影响；
  真机 launch 不再 spawn `gripper_controller`。mock 分支仍用 `parallel_gripper_action_controller`，改为
  `allow_stalling: true`，使"夹住 = 成功且 stalled"与真机一致
- 插件节点在 `<ns>/gripper_controller/gripper_cmd` 提供 `control_msgs/action/ParallelGripperCommand`
  （相对名，解析结果与 mock 相同），放在现有的 `gripper_group_` 回调组，一次执行一个 goal；执行在回调内阻塞，
  测试改用多线程执行器
- `~/gripper/activate` 与 goal 共用 `runGripper(op)`：
  1. 立即检查：已配置、帧新鲜、无故障、无保护停止；activate 要求**未激活**（已激活则拒绝：激活必然张开到满行程，G2）；
     move 要求已激活且 `gripper_fault == 0`
  2. CAS `gripper_busy_: Idle → Busy`（此后 `write()` 扣住机械臂运动）
  3. 等待，每 20 ms 查一次，上限 2 s：机械臂关节速度 < 0.5°/s，且伺服流停发（或硬件未激活），且帧中
     `servo_queue_len == 0`（G8）。超时则释放 busy 并拒绝。上限按真机实测的"JTC SUCCEEDED → 停发"时长复核
  4. 在 `api_mutex_` 下调用 `MoveGripper(block=1)` / `ActGripper`
  5. 完成条件按操作区分，每 20 ms 轮询帧：
     - activate：active 位置位且 `gripper_fault == 0`，3 s 超时（现逻辑）
     - move：先见 motiondone = 0 再见 = 1；1.5 s 内未见运动开始按"无需运动"；30 s 超时；期间发布 feedback（开度）
     - 两者：`gripper_fault` 非 0、帧过期 → 失败
  6. move 的结果：`reached_goal = |开度 − 目标| ≤ 1 %`，`stalled = 完成 && !reached_goal`（夹住，成功）；释放 busy
- cancel 一律拒绝（夹爪运动停不下来）；goal 中的速度/力按现有换算转为百分比，未给时默认 50 % / 20 %
- `write()` 的扣臂逻辑保留，补上超时（修 M0）：
  - 夹爪忙（含第 3 步的等待）且机械臂命令变化 → 扣住（命令替换为停住位置，`arm_held_`，WARN 一次）；夹爪忙期间不计时
  - 夹爪结束后，命令回到停住位置 0.01° 以内即放行（现规则）
  - 夹爪结束后 2 s 仍未回到 → halt + DEACTIVATE，ERROR 日志
  - JTC 加路径容差（`controllers.yaml` 各关节 `trajectory`），让被扣住、不跟随的轨迹尽快 ABORTED、命令回到停住位置，
    而不是跑到终点再等 `goal_time`；这本身也是"机械臂没跟上"的通用保护。取值 = 各关节 URDF 最高速度 × 实测跟随滞后
    （prefill × cmdT + 往返，约 30–40 ms）× 2，实施时列出各关节数值，并在真机正常轨迹上确认不误报。
    只有极慢的轨迹（速度低于 容差 / 夹爪时长）仍会落到 2 s 后的保护停止
  - `step_fault_` 泛化为"驱动自设的保护停止 + 原因"（步长超限 / 扣臂超时），`~/status` 报原因，重新激活清除
- 删除：`GripperSender`、arming 与 `perform_command_mode_switch`、`writeGripper`、死区、`stream_idle_frames_`、
  `readGripper` 的 busy 跟踪成员、真机的夹爪命令接口；20003 连接由 3 条减为 2 条。保留开度推断（四种来源）与速度估计、
  伺服流空闲停发（S18，现在只是 `runGripper` 的等待条件）

**W4 XML-RPC：xmlrpc-c 编解码 + libcurl**
- `RobotApi::call` 用 `xmlrpc_c::xml::generateCall` / `parseResponse`（它可能抛 `girerr::error`，转为 Protocol 错误）；
  HTTP 用一个 libcurl easy handle（POST `/RPC2`，`CURLOPT_TIMEOUT_MS` / `CURLOPT_CONNECTTIMEOUT_MS`），同一 handle 复用即
  持久连接。超时语义与现状相同（伺服流 100 ms），`halt()` 不中断进行中的调用（中断会被当成 COMM，改变"机器人故障 →
  DEACTIVATE"的语义）
- 删除 `xmlrpc_codec`、`HttpConnection` 及其测试；fake controller 改用 xmlrpc-c 的 `parseCall` / `generateResponse`
- 实施前在 1.33.14 头文件上确认上述函数的签名
- 仿真器（经 `flock`）确认：本仓库用到的全部方法照常工作；ServoJ 往返时间不劣于现实现；第 30 个请求后的
  `Connection: close` 与空闲 15 s 后自动重连；连接已被服务端关闭时 libcurl 是否重发请求（若会，确认只发生在请求未被
  读取的情形）
- AGENTS.md 规则 5 补一句：XML-RPC 报文由 xmlrpc-c 生成（与控制器服务端同源），方法与参数仍按 SDK 3.8.4

**W6 FK 改用 KDL**
- `kdl_parser` 从 `HardwareInfo::original_xml` 建 `<tf_prefix>base_link` → `<tf_prefix>flange` 链；删除 `kinematics.hpp`、
  其测试、xacro 拼出的 `kinematics` / `flange_offset` 参数。比较约定（mm、deg、R = Rz·Ry·Rx，R7）不变；
  接口收在一个函数里，日后换掉 KDL 只动这一处。顺带修正倾斜关节轴型号（`fairino3_mt_v6`）的 FK

**W6b `fr_status` gpio 缩到只剩 `fault_class`**（接口变更，单独提交）：mock 与真机同步改；其余字段只在 `~/status`；
`fault_handling.md` §7、README、AGENTS.md 的验证命令随之更新

**W7 文档收尾**：README（驱动、客户端库）、`fault_handling.md`、sdk_quirks 按新设计更新。驱动 README 的夹爪段写明使用约定：
机械臂与夹爪尽量不要同时命令，等一方的 action 结束再发另一方（MoveIt/MTC 默认如此）；同时命令时的后果（机械臂被扣住、
轨迹因路径容差 ABORTED、夹爪结束后 2 s 未回到停住位置则保护停止）与恢复方法（重新激活硬件）。补一条 📋：维护者经 SDK
实测 `MoveGripper` 的力参数生效（条件与数值由维护者提供）；TODO.md 增删；审查遗留的 M1（两指网格互穿 → SRDF/ACM）
与"MoveIt 执行时长监控对夹爪 goal"转入 MoveIt 计划；删除 `docs/review-agent-dev.md` 与
`docs/review-agent-dev-findings.md`

**W8 整理提交历史**（以上全部完成后）
- 在新分支 `agent/dev-clean` 上从 `main` 重建，原 `agent/dev` 保留不动，便于对照
- 按主题排成序列，大致：构建与 check → 客户端库各部件（各一个提交，直接是最终形态，不再现
  `ServoSender` 的四版）→ 驱动（生命周期与故障 → 伺服 → 状态话题 → 夹爪）→ 描述包与 launch → 文档；
  真机实测记录（sdk_quirks 📋、计划 Result）并入相应的功能提交，不单独保留 docs 提交
- 每个提交通过 check；保留原作者与 `Assisted-by` 尾注，不改作者/提交者身份；最终树与 `agent/dev` 末端
  `git diff` 为空。合入由维护者执行

**考虑过、未采纳**

- 只修 M0（扣臂超时转保护停止，约 20 行）、夹爪仍走 ros2_control：缺陷 a、b 与双重完成判断仍在
- 夹爪忙时机械臂命令一变即停机：维护者选择扣住加超时——机械臂这时本来就不动（G9），超时兜住 M0
- 夹爪做成 UR 式"控制器 + gpio 命令/应答接口 + 硬件异步执行"：更合 ros2_control 惯例、mock 也能跑，但要新增一个
  控制器插件包，硬件里仍需异步线程；插件节点已直接持有 `reset_error`、`gripper/activate`、`~/status`，
  action 放在同处实体更少
- 夹爪独立成节点（Franka 式）：与机械臂的互斥要跨进程共享状态
- 夹爪忙时照发 `ServoJ`、交给控制器排队：G9 / S16，换来 2 s 延迟的陈旧运动或卡住的调用
- 夹爪前置条件不满足即拒绝（不等待）：JTC 报成功时机械臂常仍在收敛，"轨迹 → 夹爪"会偶发失败，是对现设计的回退
- xmlrpc-c 完整客户端（Curl 传输）：整秒超时、`setInterrupt` 不能从别的线程及时中断（见 Context）
- 状态帧用 seqlock 或乒乓双缓冲：无锁，但 8 ms 一帧时与 `try_lock` 无实际差别；乒乓缓冲在读者被抢占超过一帧时
  仍会撕裂，补上序号检查就是 seqlock，代码更难读
- `SpscQueue` 换 `boost::lockfree::spsc_queue`：只省 52 行（加测试约 110 行），却给不依赖 ROS 的客户端库新增一个依赖
  （维护者同意不做）

**风险**

- W5 推翻已完成的夹爪计划的命令通路，真机要重验
- 路径容差取值过小会在正常轨迹上误报 ABORTED；过大则被扣轨迹收尾慢、更多落到保护停止。须真机确认
- MoveIt 的执行时长监控可能取消耗时约 2–3 s 的夹爪 goal（单点轨迹的期望时长约 0）：新设计拒绝 cancel，
  MoveIt 会报执行失败而夹爪照动——在 MoveIt 计划里配置夹爪控制器的执行时长或关闭对它的监控
- W4 依赖 libcurl 的持久连接与重连行为，须仿真器实测（见 W4）
- W6 的 KDL 链与现 FK 的数值差异须在真机上复核（Done when）
- 新依赖须重建镜像；漏加 `package.xml` 会在干净环境里构建失败
- W8 是历史改写：只在新分支上做，原分支保留到维护者合入后

## Result

未实施，无代码改动。第二轮独立审查指出：

- 捆在一起，几乎每项都要真机复验，代价集中在操作者身上；计划本身过密，难以审
- W5 的设计还有缺口：mock 与真机 action 行为分叉；`MoveGripper` 调用超时后若释放 busy，固件会在伺服流停下后照样执行
  （G8），重现缺陷 a——**现代码同样如此**（`readGripper` 见到失败计数增加即清 busy）；执行回调阻塞时的 executor
  线程占用要验证；1.5 s"无需运动"启发式缺依据（现代码亦有）；拒绝 cancel 与 MoveIt 执行时长监控冲突，须作为 MoveIt
  计划的阻塞性前置条件
- W4 换用 libcurl 需自行处理：`Content-Type: text/xml`（Abyss 校验）、去掉 `Expect:`、`CURLOPT_NOSIGNAL`、
  复用连接已断时的自动重发（可能重复发送 ServoJ）；fake controller 可考虑 `xmlrpc_c::registry::processCall`

维护者决定：W2、W1、W3、在现架构内修夹爪（调用失败不放 busy、M0 扣臂超时 + JTC 路径容差、文档写明不要同时命令）、
W6b 与提交整理放在合入前；W5（夹爪 action）、W6（KDL）、W4（xmlrpc-c）合入后各立计划；SpscQueue 不换。
本计划的 Context 仍是这些后续计划的依据。

- Revisit if: 合入后启动 W5 / W6 / W4 时，从本计划的 Context 与上述审查要点出发
