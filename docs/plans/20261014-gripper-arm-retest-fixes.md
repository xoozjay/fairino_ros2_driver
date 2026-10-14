# 让夹爪不再干扰机械臂：轨迹暂停代替扣臂停机，修正夹爪完成判定与过期命令

- Status: approved

## Goal

[20261013-pre-merge-cleanup.md](20261013-pre-merge-cleanup.md) 的真机复测（2026-10-14，3.8.4.2，FR5，PGI-140，
J6 ±30°）发现：夹住物体后机械臂与夹爪被锁 30 s；被中止的夹爪目标十几秒后仍会执行；扣臂机制依赖的路径容差按
未实测的 40 ms 滞后取值，实际滞后 118 ms；夹爪运动中收到机械臂轨迹，驱动会保护停止机械臂。合入前完成（维护者：main 应当是直接可用的驱动）。

原则（维护者）：**外设不应干扰本体**。夹爪最多让机械臂的轨迹推迟，不得让机械臂停机或轨迹失败。控制器本身不允许
`MoveGripper` 与 `ServoJ` 同时执行（G8、G9），所以二者只能轮流动；轮流由驱动安排，对上层表现为轨迹暂停、夹爪等待。

## Done when

- `scripts/check.sh` 通过，新增测试覆盖：夹爪运动期间收到的轨迹暂停、夹爪完成后继续并 SUCCEEDED（无保护停止）；
  action 目标：报 1/2 的结果与 `stalled` 语义、等机械臂期间不超时、新目标取代旧目标、取消；`motion_done = 2` 结束
  夹爪运动；原地 `MoveGripper` 期间开度不报 0
- 驱动代码中不再有因夹爪而起的保护停止（`ProtectiveStop::HoldTimeout` 删除）
- 真机（操作者发起）：夹爪运动中发 J6 +2°/4 s 与 0 → +30° 轨迹，均暂停、随后完成且 SUCCEEDED；夹住物体后 `gripper_busy` 在帧报完成时清零，随即能张开；夹爪目标被中止后夹爪不再动；
  J6 在维护者允许的最高速度（180°/s）下正常轨迹不因路径容差中止
- sdk_quirks 增加本次实测（下文 E1–E5），README 夹爪段与故障表同步

## Context

真机会话日志在会话机器的 `/tmp/session/`（不入库）；以下为关键数据。

**E1 跟随滞后 118 ms，为常数**。J6 余弦轨迹，`joint_trajectory_controller/controller_state` 的 reference 与
feedback 互相关：30°/s 最大误差 3.7°、60°/s 7.4°、90°/s 11.1°，滞后均为 116–121 ms（含 +2°/4 s、+10°/5 s 的慢轨迹）。
`controllers.yaml:44-45` 的路径容差 0.26 rad 按"prefill × cmdT + 往返，约 40 ms"算出，此值从未实测。按 118 ms，
0.26 rad 在约 126°/s 以上误报 ABORTED；误报后 JTC 保持在实际位置，命令一步跳回约 15°，单周期步长检查拒绝 →
高速中保护停止。未在真机上跑 > 90°/s。滞后的组成未知：已知部分为主机端 `kLead` 3 条（24 ms）与固件队列
`prefill` 3 条（24 ms），其余约 70 ms 待测。

**E2 小幅慢速轨迹撞上夹爪运动 → 保护停止**。夹爪 100 → 50 % 期间发 J6 +2°/4 s：被扣住；参考只偏 2°，到不了路径容差；
夹爪结束后 2 s 内命令未回到停住位置 → `HoldTimeout` 保护停止（`fr_robot_hardware_interface.cpp:790`）。
夹爪计划 Result 中同一操作的记录（goal_time 中止，之后同一轨迹成功）不再成立。大幅轨迹（0 → +30°，30°/s）则按设计
因路径容差 ABORTED、放行，无保护停止。

**E3 夹住物体时 `gripper_motiondone = 2`**。夹爪 50 % 处放物体后命令 0：位置 50 → 45 → 23 → 7 %，帧报 `motion_done = 2`
并保持。SDK 文档只写 0/1（`references/fr_documentation/source/SDKManual/CPPDataStructure.rst:204`）；大寰 PGI 自身的
夹持状态寄存器为 0 运动中 / 1 到位 / 2 夹住物体 / 3 物体掉落，推测控制器原样转发（🔍 3 未见）。驱动只认 1 为完成
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

## Approach

1. **完成判定与夹爪 action**（此前"合入后"的计划提前；维护者选 A）：夹爪 action 改由驱动插件的节点提供，类型仍为
   `control_msgs/action/ParallelGripperCommand`，替代 `parallel_gripper_action_controller`。原因：后者只看开度速度
   （上游 `parallel_gripper_action_controller_impl.hpp` 的 `check_for_success`），分不清"夹住物体"与"在等机械臂停下"
   （后者也会在 `stall_timeout` 后被判为卡住），且中止/成功都把目标留在命令接口里（E4 的根因；`accepted_callback`
   只写实名，不撤旧值）。做法：
   - 完成判定直接用帧："先报 0、再报非 0"结束——报 1 到位，action SUCCEEDED；报 2 夹住，SUCCEEDED 且
     `stalled = true`（维护者确认）；报 3 掉落，ABORTED；等机械臂停下的目标继续等，不算卡住（机械臂优先）；
     发出 30 s 仍无完成的按超时 ABORTED。1.5 s"从未开始"兜底保留（E5：起始的 0 一定出现，兜底只是防线）
   - 新目标取代旧目标（与上游一致）；取消时若尚未发出则干净取消，已发出则报取消、运动让其走完（控制器无法中止
     已发出的夹爪运动）
   - 到当前开度的目标也照发（再次夹紧是合理用法；E5 的短暂 0 不外泄由第 3 步保证）
   - 真机管线不再加载 `gripper_controller`（launch 只在 mock 下加载）；launch 把插件的 action 重映射为
     `/<ns>gripper_controller/gripper_cmd`——mock（仍用上游控制器）与真机对上层同名同类型
   - 武装机制删除（`~/status` 的 `gripper_commands` 随之删除）：action 命令只来自被接受的目标；
     `~/status` 增加 `gripper_object`（`none` / `held` / `dropped`，按 2/3）
2. **过期的夹爪命令**：随第 1 步消失——命令只来自被接受的目标，目标被取代或取消后驱动不再持有其值，上游控制器
   "中止后命令仍留在接口里"的行为不再有机会发生
3. **原地运动的开度**：发出的目标非 0 时，帧的 0 % 只在出现过非 0 位置之后才采信；否则保持运动前的开度
4. **轨迹暂停代替扣臂**（JTC 速度缩放）
   - 插件导出状态接口 `<tf_prefix>speed_scaling/speed_scaling_factor`（与 UR 驱动同名）：平时 1.0，夹爪运动期间（从发出
     `MoveGripper` 到帧报完成）0.0。`joint_trajectory_controller` 配 `speed_scaling.state_interface` 指向它（本机 Jazzy 的
     JTC 4.42.1 已支持，`joint_trajectory_controller_parameters.hpp:91`），因子为 0 时轨迹时间不前进：参考停在原地，
     不偏离、不触发路径容差，夹爪完成后从原处继续。mock 分支声明同名接口，恒为 1.0
   - 夹爪只在机械臂静止时发出（现有：伺服流停发且队列空），所以因子降为 0 时机械臂本来就停着，不会急停
   - 删除 2 s 扣臂超时与 `ProtectiveStop::HoldTimeout`。夹爪运动期间仍不把会动的命令发给控制器（控制器也不会执行，
     G9）；夹爪结束后命令须回到停住位置（0.01°）才放行——不认速度缩放的控制器属于误用，README 写明，极端跳变由
     现有的单周期步长检查兜底。`~/status` 的 `arm_commands` 保持键名不变，值增加 `paused (gripper moving)`
   - 先用测试确认 JTC 在因子 0 时的行为（参考冻结、`goal_time` 按缩放后的时间算、恢复后 SUCCEEDED），再改驱动
   - 路径容差按实测滞后重算：最快关节 3.2 rad/s × 0.118 s × 2 ≈ 0.76 rad；它只负责真正的跟随失败
   - 118 ms 的组成（已知 `kLead` 24 ms、`prefill` 24 ms，其余约 70 ms）用 `fr_servo_bench` 测清，减少滞后另立计划
   - 机械臂运动期间收到的夹爪目标照旧等机械臂停下（G8）；等待超过 `stall_timeout` 的处理见第 2 步
5. **状态**：停用时清除夹爪运动中的暂停状态
6. **文档**：sdk_quirks 增加 E1（S19）、E3（G13）、E4 的驱动侧对策、E5（G14）；README 夹爪段写明夹住物体时的行为与
   SUCCEEDED/`stalled`；故障表与路径容差注释更新

**考虑过、未采纳**：
- 保持"扣住 + 2 s 保护停止"，或夹爪忙时收到会动的命令立即保护停止：违反"外设不干扰本体"
- 路径容差放大或设为 0 来减少误报：被扣住的轨迹仍只剩保护停止，且失去对真正跟随失败的检测
- 不经运动队列控制夹爪、让二者真正同时动（末端 485 寄存器写 `AxleSensorRegWrite`，或夹爪 Modbus 直连主机）：
  未验证，可能与控制器对夹爪的轮询冲突，单列 TODO

MoveIt 的执行时长监控会把暂停的时间算作超时：MoveIt 计划须相应配置（该计划已把"夹爪 goal 的执行时长监控"列为前置
条件，机械臂轨迹同理）。

**风险**：`motion_done = 3`（物体掉落）未实测；夹爪 action 的目标回调与结果发布共用插件节点的 executor 线程，
不阻塞实时循环（参考 JTC：实时侧只用无锁缓冲）；mock 仍用上游夹爪控制器，其卡住按开度速度判定、语义与真机不同
（README 写明）；`gripper_controller` 的 `max_velocity`/`max_effort` 命令接口保留（mock 需要），真机上无人占用；
JTC 速度缩放在 Jazzy 是较新的功能，行为以第 4 步的测试为准。

## Result

（未开始）
