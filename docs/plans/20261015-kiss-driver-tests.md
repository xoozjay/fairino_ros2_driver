# 夹爪判定抽成纯逻辑、假控制器回归朴素、测试回放真机帧序列

- Status: done（第 1–5 步全量完成；真机开、合、夹住、释放及物块受力截流判定全部实测通过）

## Goal

驱动、假控制器与测试里，夹爪部分的复杂度主要来自对固件怪癖的应对与模拟：插件 `readGripper` / `gripperOpening`
用十几个成员推断"运动是否结束、该报多少开度"；假控制器为了覆盖这些推断，一个个模拟怪癖（G10、G14、G17），
每个怪癖一组定时器与开关，且只是我们对固件行为的猜测，没有与真机核对。

按 KISS 收敛：怪癖只在一处处理（一个无时钟、无 I/O 的纯逻辑），用**真机录到的帧序列**测试它；假控制器只做朴素的
"动 N ms 后到位或夹住"；插件只负责接线。行为不变，除 Questions 中维护者的决定（夹爪速度恒报 0、只留一个暂停测试）与夹爪速度参数无效（Approach 4）。

## Done when

- `scripts/check.sh` 通过
- 夹爪的完成判定与开度推断全部在 `include/fr_robot_driver/gripper_motion.hpp`（纯逻辑，时间由调用方传入）；插件中
  `gripper_seen_running_`、`gripper_sent_position_`、`gripper_frame_state_*`、`gripper_known_*`、`gripper_awaiting_nonzero_`、
  `gripper_was_active_`、`gripper_reported_`、`gripper_velocity_`、`gripper_changed_at_`、`gripper_posted_at_` 不再存在
- 新测试 `test/test_gripper_motion.cpp`：每张帧序列表注明来源（真机日期与试验名），至少覆盖下文 Context 的
  R1–R7；毫秒级，不睡眠
- 假控制器不再有 `gripper_stale_*`、`gripper_zero_after_done_ms`，MoveGripper 时不把开度置 0（G10）、激活时不报 0 %（G5）
- 插件测试中只为怪癖而设的用例删除（由纯逻辑测试取代，见 Approach 3）
- 有效性：把纯逻辑中各条件逐一关掉，对应的表格用例失败
- 真机（操作者发起，仅夹爪，机械臂不动；AGENTS.md 规则 5）：经 action 开、合、夹住、夹住后抽走再张开（从 3 张开）各一次，
  结果与 `gripper_busy` 的结束时刻与本计划一致——验证 731cdc5 的完成判定（尚未上真机）
- Result 记录改动前后的行数（插件、假控制器、插件测试）

## Context

**行数（改动前，8ce7478）**：`fr_robot_hardware_interface.cpp` 1155，夹爪段约 300 行（文件末尾），另散在 `on_init`、`on_activate`、
`read`、`write`；`test_hardware_interface.cpp` 1243，其中夹爪约 600；`fake_controller.hpp` 428，23 个 atomic，其中 10 个只为夹爪。

**怪癖在插件里的样子**：`readGripper` 同时做三件事——判定运动结束（0 之后非 0 且 300 ms 不变、目标非 0 时开度非 0，
G13、G17）、推断要报的开度（帧报 0 % 时保留旧值或报全开，G5、G10、G14）、估计开度速度（帧约每 0.4 s 跳一次）。
三者共享 `gripper_awaiting_nonzero_` 等状态，散在 `readGripper`、`writeGripper`、`gripperOpening`、`on_activate`
四处设置。时间取自 `steady_clock::now()`，只能在墙钟下测。

**可回放的真机帧序列**（会话日志，不入库；表格只抄 `motion_done`、开度、时刻）：

| # | 来源 | 内容 |
|---|---|---|
| R1 | 2026-10-14 X4 | 100 → 50 %，1 → 0 → 1 |
| R2 | 2026-10-14 E5 | 原地运动 50 → 50 %：0、完成后约 200 ms 仍报 0 %（G14） |
| R3 | 2026-10-14 X2 | 合到 0 %：夹住报 2；抽走后 0 → 3；从 2 张开 2 → 0 → 1 |
| R4 | 2026-10-15 A | 合到 10 %：28 % 报 2；抽走 2 → 0 → 3，停在 10 %；只回放帧顺序，抽走时刻记录有误，不用来推断 `max_time` |
| R5 | 2026-10-15 | 从 3 张开：0、旧的 3 带 0 % 与 14 % 再现约 0.4 s、再 0、到位报 1（G17） |
| R6 | 2026-10-15 B″ | 夹住报 2 后长时间不变（时限后抽走无变化） |
| R7 | 2026-10-14 激活 | 激活后帧报 0 %、0，直到第一次运动（G5） |

**为什么现在做**：维护者结论（2026-10-15）——夹爪命令不与机械臂命令同时发，真机不测并发；`motion_done` 的含义
已定（G13）。夹爪逻辑不会再因新实验大改，适合定形。

## Approach

每步一个提交，各自通过 check。

1. **`GripperMotion`**（`include/fr_robot_driver/gripper_motion.hpp`，与 `status_report.hpp` 同样是纯头文件）：
   - 输入：`sent(target_percent, t)`（MoveGripper 发出）、`frame(active, motion_done, position, t)`（每次 `read()`）
   - 输出：运动是否结束与结果（Reached / Grasped / Lost / Other / NeverStarted / TimedOut）、要报的开度 [m] 与
     `GripperSource`。开度速度恒报 0（维护者，Questions 2），估计逻辑删除
   - 插件的 `readGripper` 只剩：喂帧、写状态接口、交出结果；`gripperOpening` 删除。`busy`、目标与 action 的线程交接
     （`gripper_busy_`、`gripper_finished_`、`gripper_epoch_`）留在插件，不动
2. **纯逻辑测试**：`test/test_gripper_motion.cpp`，表格驱动，R1–R7；每张表一行一帧（时刻 ms、active、done、开度），
   断言结束时刻、结果与每帧报的开度
3. **假控制器回归朴素**：MoveGripper → `done = 0`、开度不变；`gripper_motion_ms` 后开度 = 目标（或物体处）、`done` = 1 或 2。
   ActGripper → active 位、开度 100 %、`done = 1`。删除 `gripper_stale_*`、`gripper_zero_after_done_ms` 及其分支。
   插件测试删除 `TheOpeningIsHeldWhileTheFrameSaysZero`、`AMotionToWhereTheGripperIsNeverReportsZero`、
   `ThePreviousFlagsComingBackAreNotTheEnd`；其余用例中只为怪癖的断言（如 `inferred_open_after_activation`）移到第 2 步。
   `test_trajectory_pause` 只留 `ATrajectoryThatMeetsAGripperMotionWaitsAndSucceeds`（维护者，Questions 3）
4. **插件小清理**（只做下列，不扩大）：
   - `on_init` 中两个 gpio 与夹爪关节的接口检查合为一个查找函数
   - **夹爪速度参数无效**（维护者，sdk_quirks G12 更正）：goal 的 `velocity` 忽略，`MoveGripper` 发固定速度 50 %；删除
     `kGripperSpeedPerPercent` 与速度换算；xacro 删除夹爪关节的 `max_velocity` 命令接口，`controllers.yaml` 中 mock
     `gripper_controller` 的 `max_velocity_interface` / `max_velocity` 随之删除
   - README 夹爪段按新的分工更新（怪癖处理在哪、速度无效、`gripper_position_source` 的含义不变）
5. **真机核对**（操作者）：见 Done when

**不在本计划**：伺服发送、状态接收、故障分类、生命周期的结构；两个墙钟偶发失败的测试（TODO 已知风险，另行处理）；
FK/KDL、XML-RPC 换库（合入后计划）。

## Questions（维护者已答复，2026-10-15）

1. ~~目标 0 % 的漏洞~~ 维护者：**不补**。已知局限：合到 0 % 时，若旧标志位带 0 % 再现超过 300 ms，会提前判定结束
   （真机未见过）；README 写明
2. ~~夹爪开度速度~~ 维护者：**恒报 0**，删除估计逻辑（`GripperMotion` 不输出速度）
3. ~~`test_trajectory_pause`~~ 维护者：**只留一个**——"夹爪运动中来了轨迹 → 暂停后完成"

## Result

基线 `3fe1abe`：`scripts/check.sh` 通过，194 tests，0 errors，0 failures，4 skipped。

**第 1–4 步完成**，各步全量 check 通过：

- 第 1 步 `509aa5b`：`GripperMotion` 接管运动结束、开度与来源推断，插件只喂帧与交出结果；开度速度恒报 0。
  原有 194 tests 通过
- 第 2 步 `319b405`：新增 13 项纯逻辑测试，约 2 ms，check 共 208 tests。R1–R7 来自黑盒日志与已记录的实测表；
  原始时刻四舍五入到 ms，另补 300 ms 稳定期边界的断言，在两条变化记录之间逐 ms 重复上一帧。
  R4 只回放帧顺序，不用有误的抽走时刻推断 `max_time`。正常夹住后 action 已结束，之后的 2 → 0 → 3 不产生第二个结果。
  另以纯逻辑边界测试覆盖未开始、未完成、未知标志位、重新激活与重置，不等待真实超时
- **测试有效性**：在临时目录逐一关闭 10 处条件（开始信号、非零标志、稳定期、位置可信性、位置变化重置计时、
  忽略命令前开度、零位置保持、激活后丢弃旧开度、1.5 s 与 30 s 截止），对应测试全部失败；源码未改。
  未新增目标 0 % 的防护或 H1 假设用例
- 第 3 步 `1ac7674`：假控制器删除旧标志再现与延迟零位置的状态、开关、定时器；运动中保持开度，完成时直接报目标或物体位置；
  激活后报全开。删除三项怪癖插件测试，`test_trajectory_pause` 只保留一个用例。check 共 203 tests，0 failures
- 第 4 步 `b26d448`：goal 速度忽略，协议值固定 50；删除 `max_velocity` 命令接口与 mock 控制器的对应配置，力参数保持换算。
  三处接口检查共用查找函数；README 写明速度状态恒 0、顺序执行、目标 0 % 的已知局限与时限后物体不可观测。
  最终 `scripts/check.sh`：203 tests，0 errors，0 failures，4 skipped；纯逻辑测试 13 项。
  mock 启动与开合验证使用的代码与最终树相同。

**mock 管线**（独立 ROS domain、namespace 与 tf_prefix，使用安装后的 launch / xacro / controllers.yaml）：
三个控制器 active；夹爪仅 position / max_effort 命令接口；指定 goal.velocity 后开到 0.04 m、合到 0 m 均 SUCCEEDED，
`joint_states` 与目标一致；随后单独发机械臂轨迹 SUCCEEDED。验证进程已退出。

| 文件 | 重构前行数 | 重构后行数 |
|---|---:|---:|
| `src/fr_robot_hardware_interface.cpp` | 1155 | 1062 |
| `include/fr_robot_driver/fr_robot_hardware_interface.hpp` | 218 | 202 |
| `include/fr_robot_driver/status_report.hpp` | 257 | 252 |
| `include/fr_robot_driver/gripper_motion.hpp` | — | 116 |
| `test/fake_controller.hpp` | 428 | 390 |
| `test/test_hardware_interface.cpp` | 1243 | 1158 |
| `test/test_trajectory_pause.cpp` | 261 | 219 |
| `test/test_gripper_motion.cpp` | — | 261 |

生产代码总行数基本持平；测试总行数增加，新增内容主要是带来源的帧表。简化的是状态归属与测试时序：插件的分散判定
集中到一个纯逻辑对象，假控制器少了四个夹爪配置项及其定时分支，怪癖无需再经 socket、线程和墙钟模拟。

**实施与核对全量完成（2026-10-15）**：

第 5 步真机核对已在真实 PGI-140 夹爪上由操作者交互验证完成：
- 经 `/fr_robot_hw/gripper/activate` 触发上电使能与零位标定；
- 经 `/gripper_controller/gripper_cmd`（`ParallelGripperCommand`）执行张开（0.04 m）、空闭合（0.0 m）以及多次不同尺寸物块夹取（开度 22.4 mm、7.2 mm）；
- `readGripper` 与 `GripperMotion` 对到位（`reached_goal=True, stalled=False`）与夹住受力截流（`stalled=True, reached_goal=False`）的推断 100% 吻合物理实际；
- 夹爪动作开始与结束期间 `gripper_busy` 准确翻转，机械臂轨迹调度互锁行为正常。计划至此全项闭环。
