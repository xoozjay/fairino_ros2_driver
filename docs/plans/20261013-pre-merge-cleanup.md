# 合入前收敛 agent/dev：修并发与夹爪缺陷、删死代码、整理提交

- Status: approved

## Goal

`agent/dev` 合入 `main` 前，只做小而确定的改动：修掉对象生命周期竞态与两处夹爪缺陷，删除不用的代码，状态帧只留一条
交接路径，冻结 `fr_status` 接口，然后把提交历史整理成可读、可 bisect 的序列。架构不变；夹爪改 action、FK 改 KDL、
XML-RPC 改 xmlrpc-c 合入后各立计划（TODO.md）。问题分析与证据见被放弃的
[20261013-kiss-refactor.md](20261013-kiss-refactor.md) 的 Context，本计划不重复。

## Done when

- `scripts/check.sh` 通过，且：
  - W2 所列代码与 `TripleBuffer` 不再存在
  - 新增常驻测试：多线程执行器下 status 定时器运行中反复 configure / cleanup
  - 新增夹爪测试：`MoveGripper` 调用失败后机械臂仍被扣住，直到帧显示夹爪空闲或完成；夹爪结束后命令 2 s 内未回到停住
    位置 → 保护停止，`~/status` 写明原因
- 一次性 ThreadSanitizer 构建跑两包测试，本仓库代码无数据竞争报告
- 真机（操作者发起，AGENTS.md 规则 5）：
  - 夹爪计划 Result 的运动与轮流动项重跑
  - 夹爪运动中发机械臂轨迹：被扣住 → 轨迹因路径容差 ABORTED → 放行；以及不放行时 2 s 后保护停止
  - 路径容差在正常轨迹上不误报（含该台机器允许的最高速度）
  - `MoveGripper` 到当前开度时帧里 motiondone 是否出现 0（给 1.5 s 启发式定依据）
  - 拔网线 → 重新 configure → activate 不崩溃
- 整理后的分支：提交映射表经维护者批准；每个提交通过 check；最终树与整理前的 `agent/dev` 相同；维护者合入

## Approach

每项独立提交、通过 check；顺序即编号。

1. **删除未用与越权代码**：`RobotApi` 的 `robotEnable`、`setMode`、`setSpeed`、`dragTeachSwitch`、`setLoadWeight`、
   `setLoadCoord`、`setToolCoord`、`setDO`、`setToolDO`；`InstantCommand` 的 Pause/Resume/ExtAxisStopJog/
   ConveyorComDetectTrigger；`ServoSender::resume` 及 `resume_mutex_`/`applyResume`；`StateReceiver::error()`；
   `getGripperConfig`（调用与 API）；连同单测与 fake controller 分支

2. **生命周期与线程兜底**
   - `StateReceiver`、`ServoSender`、`GripperSender` 在 `on_init` 创建一次，不再重新赋值；软件版本改为
     `StateReceiver::start(version)` 的参数，ServoSender 的回调选项在 `start()` 传入；`fr_probe`、`fr_servo_bench`
     随之改。去掉 `receiver_mutex_` 与 `shared_ptr` 续命
   - ServoSender、GripperSender、StateReceiver、cleanup、`monitorLink` 的线程入口包 `catch (...)`：发送与接收线程转成
     传输错误（走已有的 COMM 路径），cleanup / monitor 记 ERROR 后结束
   - 测试见 Done when；TSan 用独立的 `--build-base` / `--install-base`，ROS/DDS 误报用临时 suppression 文件（不入库）

3. **状态帧单一路径**：`StateReceiver` 只留 mutex 保护的最新帧，`latest()`（阻塞）与 `tryLatest()`（`try_lock`）；
   `read()` 与 ServoSender 的两个回调用后者，拿不到沿用上一帧（过期仍由 `comm_timeout_` 判定）。删除 `TripleBuffer`。
   新鲜度判断收成一个函数；"帧分类 + 驱动自设的保护停止"收成一个函数，`read()` 与 `StatusTracker` 共用

4. **夹爪：在现架构内修两处缺陷**
   - 调用失败不放 busy：`MoveGripper` 失败（含超时）后保持 `GripperBusy::Moving`，直到帧显示完成（motiondone 先 0 后 1）、
     或长时间未见运动（与"从未开始"同一判定）、或 30 s 超时——超时的调用会在伺服流停下后被执行（G8）
   - M0 扣臂超时：夹爪忙期间扣住不计时；夹爪结束后命令 2 s 内未回到停住位置（0.01°）→ halt + DEACTIVATE。
     `step_fault_` 泛化为"驱动自设的保护停止 + 原因"（步长超限 / 扣臂超时），`~/status` 报原因，重新激活清除
   - JTC 路径容差：`controllers.yaml` 各关节 `trajectory`，让被扣住、不跟随的轨迹尽快 ABORTED，命令回到停住位置。
     取值 = URDF 最高速度 × 跟随滞后（prefill × cmdT + 往返，约 40 ms）× 2；FR5 为 3.15 rad/s → 约 0.25 rad，
     与 UR 官方配置的 `trajectory: 0.2` 同量级（`ur_robot_driver/config/ur_controllers.yaml`）。各型号共用一份配置，
     取最快型号的值
   - 驱动 README 夹爪段写明：机械臂与夹爪尽量不要同时命令，等一方 action 结束再发另一方；同时命令的后果与恢复方法

5. **`fr_status` gpio 缩到只剩 `fault_class`**（接口变更）：mock 与真机同步；`fault_handling.md` §7、README、
   AGENTS.md 验证命令随之更新

6. **文档与待办**：sdk_quirks / README 与以上改动同步；补一条 📋：维护者经 SDK 实测 `MoveGripper` 的力参数生效
   （条件与数值由维护者提供）；向 MoveIt 计划提出修订（需维护者再批）：两指网格互穿须在 SRDF/ACM 中禁用（审查 M1）；
   夹爪 goal 的执行时长监控须先解决（阻塞性前置条件）；删除 `docs/review-agent-dev.md` 与
   `docs/review-agent-dev-findings.md`；TODO.md 更新

7. **整理提交历史**（1–6 完成后）
   - 先出映射表供维护者审：`agent/dev` 每个提交 → 保留 / 合并到某提交 / 修改说明 / 丢弃，及新序列的提交说明草稿
   - 原则：每个新提交是一个关注点、直接是最终形态；修复并入引入问题的那个提交（1–5 的改动也按此折回，例如死代码从未
     加入、生命周期从一开始就正确）；`ServoSender` 的四版只留最终形态；真机实测记录与计划并入相应功能提交；
     加了又删的内容（如审查指南）连同其删除一起丢弃
   - 在新分支 `agent/dev-clean` 上做，`agent/dev` 保留不动；每个提交通过 check；最终树与 `agent/dev` 末端
     `git diff` 为空；保留原作者与 `Assisted-by` 尾注，不改作者/提交者身份；合入由维护者执行

**考虑过、未采纳**：见 [20261013-kiss-refactor.md](20261013-kiss-refactor.md) 的"考虑过、未采纳"与 Result。
本计划另外不做：夹爪忙时机械臂命令一变即停机（维护者选择扣住加超时）；用 `git reset --soft` 按路径分组重新提交
（维护者要求逐提交合并、修改、丢弃）。

**风险**

- 路径容差过小会在正常轨迹上误报 ABORTED，过大则被扣轨迹收尾慢、更多落到保护停止；须真机确认
- 第 7 步折回修复会改动早期提交的内容，冲突多；映射表先批、逐个提交跑 check 控制风险；原分支保留可对照
- 现架构下夹爪 action 的缺陷 b（cancel 后再发一条）与双重完成判断仍在，留给合入后的夹爪 action 计划

## 第 7 步：提交映射表（维护者已批准）

范围 `main..agent/dev`（76 个提交，含本计划第 1–6 步的 7 个）。新序列 27 个提交，按原顺序排列以减少冲突；
每个新提交直接是该关注点在那一刻的最终形态；作者均为同一人，合并的提交保留各自的 `Assisted-by` 尾注（去重）。

**新序列**（← 来源；未注明者即原样保留）：

| # | 新提交说明（草稿） | 来源 | 修改 |
|---|---|---|---|
| N1 | `docs: add SDK quirks, fault handling design and version constraints` | d1fdc08、af56af2 | af56af2 的 TODO 部分移到 N2；C7 对策写成现状（7f99812 的 C7 改动折回） |
| N2 | `docs: add TODO.md with open items, unverified claims and risks` | 437a95d、e6f27bc | 工作日志与客户端库计划（先加后删）不进历史；RViz 渲染一项（437a95d 加、4b96bc2 删）丢弃 |
| N3 | `feat(client): add fr_client_library skeleton and RobotState384 layout` | ce06434 | — |
| N4 | `feat(client): parse software versions, whitelist and map state frame sizes` | ee6dcc0、36c0dec | 修复折回：第四位起视为同一版本从一开始就成立 |
| N5 | `feat(client): add the state frame parser and state receiver` | d98e1dc、5072d93（snapshot 部分）、7f3c258、8ebb5c4（接收器部分）、44f22f3（`error()`） | 无 TripleBuffer：一个加锁的最新帧，`latest()`/`tryLatest()`，`start(version)`，重启后忘掉旧帧，线程兜底 |
| N6 | `feat(client): add the 8080 instant command channel (STOP only)` | bdc0a63、44f22f3（8080 部分） | 只有 STOP，无计数器 |
| N7 | `feat(client): add XML-RPC codec, keep-alive HTTP, RobotApi and fr_probe` | 3a0c0d6、0c81cc9、e81d09f | 只有驱动用到的调用（44f22f3 折回）；fr_probe 直接打印夹爪字段 |
| N8 | `feat(client): classify faults from the state frame` | 5072d93（fault_state 部分） | — |
| N9 | `feat(client): add ServoSender` | d8521fd、baab48f、23c7625（库）、388b1a8、9161854（库测试）、8ebb5c4/44f22f3（发送器部分）、1da541d、5fbaba9/34f2d60（漂移补偿计划） | 只留最终形态：SPSC 队列、预填、重采样与漂移补偿、空闲停发、可选 SCHED_FIFO、无 `resume()`、线程兜底；附漂移补偿计划及其 Result |
| N10 | `feat(client): add fr_servo_bench` | 1f6fdd0、1025d71、23c7625（bench）、189160e、eefdcf1 | 最终形态（含原地 ServoJ、`--no-compensation`）；满队列实测（S16）随之 |
| N11 | `feat(client): map the controller's model string to the fr_description model` | 21548a9 | — |
| N12 | `build: treat compiler warnings as errors` | 047dac2 | — |
| N13 | `docs(client): add the fr_client_library README` | fcbccac 及之后各提交对它的修改中属于已存在部件的部分 | — |
| N14 | `chore: add scripts/check.sh and list the commands in AGENTS.md` | fb1a6d3、8b44832（check.sh 部分） | — |
| N15 | `feat(driver): add FrRobotHardwareInterface and the real-hardware launch` | 7a5460d、c32835d、8b44832/a903b0b（测试架）、9161854（驱动测试）、23c7625（驱动部分）、8ebb5c4/7f3c258/44f22f3（插件部分） | 生命周期从一开始就正确：连接对象在 `on_init` 创建一次、线程兜底、`read()` 用 `tryLatest()`；漂移补偿与空闲停发的回调直接接上；含多线程执行器下反复 configure / cleanup 的测试 |
| N16 | `feat(driver): check the controller model and flange pose against FK` | 945116c、8fceb57 | — |
| N17 | `feat: read and log the controller's payload and tool at configure` | f7c22cc、48a1a49 | — |
| N18 | `feat(driver): optional SCHED_FIFO priority for the servo sender thread` | 3a3f635 | — |
| N19 | `docs: plan and record the real-robot bring-up` | 6285a77、be315b7、83ca406、91e763c、6644e73、51a15aa、ce22ef1、d9161e7 | 计划、Result、对应的 sdk_quirks 📋 与 AGENTS.md 真机规则一起 |
| N20 | `feat(driver): publish ~/status through faults and link loss` | aa52972、cfa45cf、5fbaba9/34f2d60（故障上报计划）、e4a3618、7f3c258（`effectiveFault`） | `fr_status` gpio 在此缩为只剩 `fault_class`（其余进 `~/status`，含 `servo_queue_len`）；链路监视复用同一接收器 |
| N21 | `docs: state AGPL-3.0 in the READMEs and the gripper package` | e6d3b2f | License 待办一项（e6f27bc 加、c44ed17 删）不进历史 |
| N22 | `feat: make the PGI-140 description a xacro macro mounted on tool0` | 334a234 | — |
| N23 | `feat(client): gripper calls and a background gripper sender` | 068d79c、523b01b、f0faba3 | 无 `getGripperConfig()`、`failures()`；线程兜底；G1–G7、G11 实测随之 |
| N24 | `feat(driver): PGI-140 gripper through ros2_control` | 6f97f2f、da20127、b637bb5、4dddd34、dc0e69e、5cee437、3966ca3、e9fc53b、9529211、7f99812（README 部分） | 未激活报全开、调用失败不放行机械臂、扣臂 2 s 超时保护停止、JTC 路径容差、"不要同时命令"的约定都是初版即有；附夹爪计划及其 Result、G8–G10 与 S18 实测 |
| N25 | `feat: show the robot model when RViz opens, at screen size` | e0e4be5 | — |
| N26 | `docs: plan the MoveIt configuration and the flange circle acceptance test` | b87cb54、391d34b、46d6fcb、d5bab80、7f99812（MoveIt 计划部分） | 计划为最终形态（修订待批的 draft） |
| N27 | `docs: plan and record the pre-merge cleanup` | ab51f49 及填写本计划 Result 的提交 | 两份计划（放弃的 kiss-refactor 与本计划） |

**丢弃**（加了又删）：c44ed17（TODO 中的审查待办）、f7803d0（审查指南）、4b96bc2（删 RViz 待办）、7f99812 中删除审查指南的部分；
`docs/review-agent-dev-findings.md` 从未入库。第 1–6 步的 44f22f3、8ebb5c4、7f3c258、9529211、e4a3618、7f99812 不单独出现，
按上表折回。TODO.md、sdk_quirks、README 的逐条改动随所属的功能提交走。

**做法**：在 `agent/dev-clean`（自 `main`）上逐个构造新提交：每个新提交先按来源 cherry-pick / 合并，再把文件改成该时刻的
最终形态（后续修复折回），跑 check 后提交；全部完成后 `git diff agent/dev agent/dev-clean` 须为空。

## Result

（进行中；第 1–7 步完成，真机项待操作者，合入待维护者）

第 1–6 步各一个提交（44f22f3、8ebb5c4、7f3c258、9529211、e4a3618、7f99812），每个都通过 `scripts/check.sh`；
`rm -rf build install log` 后从零跑 check：179 tests, 0 failures, 4 skipped（基线 180 tests；删去 `test_triple_buffer` 与
8080 计数器的用例，新增接收器重启、多线程重配置、夹爪调用失败、扣臂超时 4 个）。

- **与计划的出入**：ServoSender 的两个回调仍在构造时传入（未改为 `start()` 参数）——接收器也在 `on_init` 创建一次，回调捕获的
  对象与发送器同寿命，改接口没有收益。另把 RobotApi 与 8080 通道也改为 `on_init` 创建一次；断线后的链路监视复用同一个
  `StateReceiver`（`start()` 时丢弃旧帧），不再另建，`monitor_rx_` 与 `receiver_mutex_` 随之删除。`~/status` 增加
  `servo_queue_len`（gpio 删掉后它没有别处可看）。路径容差取 0.26 rad：最快型号的关节是 3.2 rad/s（各型号 wrist，
  `fr_description/config/*/joint_limits.yaml`），3.2 × 0.04 × 2 = 0.256
- **竞态回归测试有效**（ThreadSanitizer，独立 build/install 目录，未用 suppression）：第 1 步后的旧代码上跑新测试
  `ReconfiguringWhileTheStatusTimerRunsIsSafe`，3 次均报 data race（`on_configure` 重新赋值 `gripper_sender_` 与
  `publishDiagnostics` 读它，`fr_robot_hardware_interface.cpp:412` / `:643`）；新代码上同一测试 3 次无报告
- **TSan 全量**：`fr_client_library` 全部测试无报告；`fr_robot_driver` 31 个测试通过，6 条报告全是
  `lock-order-inversion`，栈只在 `libhardware_interface.so`（`HardwareComponent::initialize`，ros2_control 内部），
  不含本仓库代码；无 data race
- **夹爪调用失败的测试有效**：把旧规则（失败即放行）临时放回，新测试 `AFailedCallKeepsTheArmStill…` 失败
  （`gripper_busy` 为 0、机械臂命令被发出）
- mock 管线：`gripper:=pgi140 mock_gpio_commands:=true` 启动，三个控制器 active，`fr_status/fault_class` 为唯一 gpio 接口；
  `joint_trajectory_controller` 参数 `constraints.shoulder_pan_joint.trajectory = 0.26`
- 维护者答复：映射表批准；`MoveGripper` 的速度与力参数都有效（记为 sdk_quirks G12，维护者确认，无另外的条件与数值）；
  夹爪动作一般 2–5 s、SDK 上限 30 s（写入 MoveIt 计划的修订）
- **第 7 步**：`agent/dev-clean`（自 `main`）27 个提交，按映射表；作者均为原作者，未改作者/提交者身份，提交日期取各提交第一个
  来源的作者日期，`Assisted-by` 取来源的并集。写入本 Result 之前，`git diff aede8ab agent/dev-clean` 为空（`aede8ab` 为整理前
  `agent/dev` 末端，保留不动）；之后两者只差本计划与 TODO.md 的这次填写。逐提交 check：`git rebase main -x` 对每个提交从
  当时的源码构建并跑当时存在的包的全部测试，27 个都 0 failures（N1–N2 16 个、N9 120 个、N15 140 个、N20 166 个、N24 起
  179 个；N14 之前用 check.sh 的等价脚本）
- **与第 7 步做法的出入**：没有逐个 cherry-pick 再改，而是逐行追溯每一行在原提交序列中的来源、按映射表归到新提交，生成
  每个新提交时刻的文件（工具未入库）。中间状态有几处须手工改成可编译：N15–N19 的插件还没有 `effectiveFault` 与
  `StateReceiver::start(version)` 之后的写法，用当时的 `classifyState`/`describeFault`；N15–N23 的伺服测试用慢斜坡代替
  `run(400)`（静止的机械臂会停发）；N20–N23 的测试架没有 `gripper:=` 参数。这些都在之后的提交里变成最终形态。
  提交说明比草稿略有改动：N12 不提尚不存在的插件；N14 去掉测试顺序执行的说明
- **偶发失败**：构造 N24 时一次 check 中 `Drift/DriftTest.QueueDepthSettlesAndThePathStaysSmooth/3`（−5000 ppm）失败
  （固件队列在第 1260 条见底）；重跑 check、单独跑 3 次与 rebase 中的那次都通过。2 ms 周期、靠 `sleep_until` 的测试
  对机器负载敏感。之后（另一提交）删除该测试：把 PI 环抽成无时钟的 `DepthLoop`，在模拟时间上用生产参数测试，
  毫秒级；它暴露了旧测试（带宽 4 rad/s）掩盖的起步排空界限，记入 TODO 已知风险
- **真机复测**（2026-10-14，3.8.4.2，FR5，PGI-140，J6 ±30°，维护者在场）：
  - 静止停发与 J6 +2°/4 s、+10°/5 s 往返：终点误差 ≤ 0.0004°，运动中队列 2–3，停下后重新停发；夹爪 50 → 100 % 期间
    机械臂偏差 ≤ 0.0006°
  - 夹爪运动中发 J6 0 → +30°（30°/s）：被扣住、因路径容差 ABORTED、夹爪完成后放行，无保护停止
  - 夹爪运动中发 J6 +2°/4 s：被扣住，参考偏差不到路径容差，夹爪完成后 2 s → 保护停止（`HoldTimeout`）；解除后正常
  - 路径容差：≤ 90°/s 不误报；实测跟随滞后 118 ms（非假设的 40 ms），预计约 126°/s 以上误报，未跑更高速
  - 原地 `MoveGripper`：`motion_done` 第一帧即为 0，1.5 s 兜底不触发
  - 新问题（夹住物体时 `motion_done = 2` 致 30 s 锁定、被中止的夹爪目标之后仍执行等）见
    [20261014-gripper-arm-retest-fixes.md](20261014-gripper-arm-retest-fixes.md)
  - 拔网线：92 ms 内报 COMM（`no state frame in time`）→ `unconfigured`，`~/status` 报 `fault_class=3`、`link=down`；
    插回后后台监视重连只读状态流，`~/status` 恢复 `link=up`；configure（FK 偏差 0.10 mm）→ activate → 控制器恢复，
    J6 +2° 往返正常，进程未崩溃。控制器管理器在停用控制器时报 "Error while attempting mode switch when deactivating
    controllers in read cycle"（ros2_control 内部，未影响恢复）
  - 已激活时再次激活（硬件 active、无故障、机械臂静止、夹爪 50 %）：被拒（`gripper 1 is already active`），未发
    `ActGripper`，`~/status` 无变化，开度保持 50 %
- 未做：合入
