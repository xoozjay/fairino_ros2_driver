# 把插件里的决策抽成纯逻辑，插件只做接线；测试改测纯逻辑

- Status: approved

## Goal

`FrRobotHardwareInterface` 把参数校验、连接前检查、机械臂命令放行、夹爪与机械臂轮流、夹爪目标的生命周期、
故障判定和 ROS 接口写在同一个类里。要测其中任何一条规则，都得起资源管理器、执行器、三条 socket 的假控制器和
发送/接收线程，按真实时间一个周期一个周期地跑：慢、怕负载，而且改一处规则要读懂整个类。

把决策搬进不带线程、不带 I/O、时间由调用方传入的纯对象（同现有的 `GripperMotion`、`DepthLoop`、`StatusTracker`），
插件只剩接线；测试改测这些对象。对外行为不变。顺带修 TODO.md"代码与测试"里的三处：激活服务持锁 3 s（下称 C4）、
夹爪发送线程退出后目标永远等待（C5）、真机上没用的夹爪命令接口（C2）。

## Done when

- `scripts/check.sh` 通过；`fr_robot_driver` 的测试总耗时 ≤ 10 s（各 `*.gtest.xml` 的 `time` 之和，现约 34 s）
- 下文"拆分"表中的每条决策都在纯对象里、有自己的毫秒级测试；`test_hardware_interface.cpp` 只剩"保留的插件测试"所列用例
- 对外接口不变：ros2_control 接口（C2 除外）、话题、服务、action、launch 与 `<hardware>` 参数，
  README 中描述行为的段落无需改动；mock 管线 `gripper:=pgi140` 启动，三个控制器 active，开合与一条轨迹各 SUCCEEDED
- TODO.md 中"插件职责过重"一项替换为指向本计划，C4、C5（及 C2）三项删除

## Context

**现状**（`main` = `174cd67`）

- `fr_robot_driver/src/fr_robot_hardware_interface.cpp` 1062 行：`on_init` 77–315（参数、节点、服务、action）、
  `on_configure` 316–455、`read()` 727–771、`write()` 772–831、夹爪 832–1039；头文件里互斥量、原子量、线程、定时器、
  服务与 action 成员共 22 个
- 驱动测试 34 s（最近一次 check）：`test_hardware_interface` 31 个用例中 14 个超过 0.9 s，最慢的是夹爪目标类
  （`GoalsSendPositionAndForceAndIgnoreVelocity` 2.7 s、`AFailedCallKeeps…` 2.6 s、`GoalsAreRejected…` 2.1 s、
  `ACanceledGoal…` 2.0 s、`AGraspSucceeds…` 2.0 s、`ANewGoalReplaces…` 1.9 s）；`test_trajectory_pause` 3.2 s
- 合入前的逐提交检查中，`dcaa328` 在负载下偶发失败一次（插件测试依赖真实时间）

**为什么不照搬 UR 把 ROS 接口移到控制器**（TODO.md 原写法）

UR 的硬件插件没有自己的节点；服务在 `ur_controllers/GPIOController` 里，经"命令接口 + `*_async_success`"握手由插件的
`write()` 执行（`ur_robot_driver/src/hardware_interface.cpp:406-521`、`ur_controllers/src/gpio_controller.cpp:421-444`）。
这依赖 UR 的故障模型：机器人故障时硬件保持 active。本驱动相反（docs/fault_handling.md）：机器人故障即 `write()` 返回
`DEACTIVATE`。在 ros2_control 4.48.1 中：

- `write()` 只在 ACTIVE 时调用（`references/ros2_control/hardware_interface/src/hardware_component.cpp:419-420`）
- 硬件 `DEACTIVATE` 时，用到其命令接口的控制器被一并停用（`controller_manager/src/controller_manager.cpp:3632-3667`）

所以 `reset_error` 若放进控制器，恰在需要它的时候（硬件因故障 inactive）既不可用、也无人执行。要走 UR 的路，得先把
故障模型改成"故障时硬件保持 active、轨迹由速度缩放冻结并中止"，重写 fault_handling 并重做真机验证——不在本计划。
夹爪 action 单独移进控制器可行（硬件停用时目标本就该中止），但夹爪与机械臂的轮流（G8、G9）必须留在硬件里，控制器只能
经接口握手搬运目标的簿记：代码更多，mock 与真机的语义差异照旧。本计划不做。

## Approach

纯对象一律：不持有线程、socket、rclcpp 对象；时间、状态帧字段、命令值作为参数传入；结果是值。代码从插件**搬移**，
不重写语义；每一步一个提交，各自通过 check。

**拆分**

| # | 纯对象（头文件） | 接管的决策（现位置） | 取代的插件测试 |
|---|---|---|---|
| 1 | `ArmGate`（`arm_gate.hpp`） | `write()` 的机械臂部分：每 `ratio_` 周期一段、夹爪忙时保持原地、单周期步长检查、"本周期机械臂要动"（给夹爪用） | `StepCheck…`（逻辑部分）、`ASmoothRamp…`、`ControllerFasterThanCmdT…`、`AControllerThatIgnoresThePause…`、`AnArmCommandWaits…` |
| 2 | `GripperGoals`（`gripper_goals.hpp`） | 目标的接受/拒绝条件、抢占、取消、epoch 作废、何时可发出（机械臂静止、伺服流停发且队列空）、结束标志位到 action 结果的映射；**C5**：发送线程不在时目标 ABORTED | `GoalsSend…`、`GoalsAreRejected…`、`ANewGoalReplaces…`、`ACanceledGoal…`、`AGraspSucceeds…`、`AFailedCallKeeps…`、`AGoalWaits…` |
| 3 | 前置条件（`preconditions.hpp`，并入现 `reset_policy.hpp`） | configure：版本白名单、型号对照、法兰核对；activate：帧新鲜、无故障、法兰；夹爪激活：已 configure、无故障与保护停止、机械臂静止、伺服流停发；`reset_error` | `Unverified…`、`AControllerOfAnotherModel…`、`AnUnknownModel…`、`AFlangePose…`、`ActivationChecksTheFlangeAgain`、`ActivationIsRefused…`、`InconsistentRates…` |

插件保留：生命周期回调的接线、`read()`（取帧、发布状态接口；判定调用纯函数）、`write()`（调 `ArmGate`、入队、交夹爪命令）、
rclcpp 的节点/服务/action/定时器、清理线程与链路监视。`reset_policy.hpp` 并入 3 后删除（文件数不增）。

**顺带修**

- C4：`~/gripper/activate` 只在调 `ActGripper` 时持 `api_mutex_`，等激活位的 3 s 不持锁
- C5：见表中 2
- C2：真机分支的夹爪关节不再声明 `position`、`max_effort` 命令接口，插件也不再要求它们；开度状态接口不变；mock 保留
  （上游 `gripper_controller` 要用）。真机上夹爪只经插件的 action 控制：插件从不读这两个接口
  （`fr_robot_hardware_interface.cpp` 对夹爪关节只有 `set_state`，851–852），真机也不加载 `gripper_controller`
  （`fr_control.launch.py:149`）。不改为"命令接口一变就发 `MoveGripper`"：接口存的是值、不是请求，插件分不清新目标与
  被中止目标留下的旧值，等机械臂停下后会执行过期命令（[20261014-gripper-arm-retest-fixes.md](20261014-gripper-arm-retest-fixes.md) E4，真机上两次）

**保留的插件测试**（假控制器，验证接线与纯对象拼起来对）：`ConfigureReadsTheRobotAndExportsItsState`、
`ActivationStartsAtTheActualPositionAndStreamsCommands`、`RobotFaultStopsTheStreamAndRecoversOnlyByTheOperator`、
`EmergencyStopRefusesAnyReset`、`SilentStateStreamIsACommunicationFault`、`LostStateConnectionIsACommunicationFault`、
`StepCheckRefusesAJumpAndTheFirmwareNeverSeesIt`、`DeactivationFlushesTheFirmwareQueueWithStop`、
`ReconfiguringWhileTheStatusTimerRunsIsSafe`、`TheStreamPausesWhileTheArmStandsAndResumesWithAPrefill`、
`StatusTopic.IsPublishedInEveryLifecycleStateAndFollowsFaults`、`MockDescriptionHasTheFingerJoint`、
`ActivationOnlyOnRequestAndOnlyOnce`，以及新写的一个端到端夹爪用例（目标 → `MoveGripper` 参数 → SUCCEEDED，期间
`speed_scaling_factor` 为 0、机械臂命令不发出）。`test_trajectory_pause` 删除（维护者：用处不确定）：它测的是上游 JTC
对速度缩放的处理；本驱动的部分（夹爪忙时因子为 0）由上面的端到端用例覆盖。

**步骤**（顺序即提交顺序）：1 → 2（含 C5）→ 3 → C4 → C2 → 删除被取代的插件测试、补端到端夹爪用例 → README 文件结构
与 TODO.md。不做真机回归（维护者：这些行为已在真机上测过）；验证靠纯逻辑测试、保留的插件测试与 mock 管线。

**考虑过、未采纳**

- 照搬 UR 把 ROS 接口移到控制器：见 Context
- 给整个插件注入时钟、保留现有测试：线程与 socket 还在，测试仍要起整套假控制器，只是等得少些
- 维持现状：测试慢且在负载下偶发失败；改一条规则要读懂整个类

**风险**

- 改动的是真机已验证的代码路径，且不再上真机复测：只搬移不改语义，每步一个提交、各自通过 check，出问题可 bisect
- 搬移时漏掉某个隐含的顺序依赖（如 `read()` 先于 `write()` 置 `speed_scaling_factor`、夹爪忙在同一周期生效）：
  纯对象的接口按"一个周期的输入 → 输出"设计，顺序由插件按现有顺序调用；保留的插件测试覆盖轮流的端到端行为

## Result

（未开始）
