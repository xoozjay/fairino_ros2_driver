# 让上层在故障与断线期间始终知道发生了什么

- Status: done

## Goal

上层（应用、监控）在任何时候都能知道机器人与驱动的状态，包括通信中断期间和恢复之后：
断了多久、为什么断（若控制器重新上线，它自己报的原因），而不是一片沉默。机器人作为工业设备
自己保证安全，驱动只负责如实上报，不做额外动作。

## Done when

- 驱动在硬件任何生命周期状态下（含 `unconfigured`）都发布状态话题，内容随故障变化
- 真机急停演练：上层先看到 COMM（含断线原因与时刻），控制器重新上线后无需操作者干预即看到
  ROBOT_FAULT 1/149；全程驱动不发任何指令
- 单测覆盖：断线 → 状态流恢复 → 报控制器故障；断线期间不调用任何 XML-RPC / 8080
- `fr_robot_driver/README.md`"故障与恢复"、`docs/fault_handling.md` 改为实测行为

## Context

- 真机急停 = 控制箱断电：驱动先报 COMM，松开后控制器自行上线并报 `main_code=1 sub_code=149`
  （`docs/sdk_quirks.md` F4）。状态帧里从未出现 `EmergencyStop=1`
- COMM 时插件 `read()` 返回 `ERROR`，ros2_control 把硬件转 `unconfigured`，**并停用使用它的所有控制器，
  包括 `joint_state_broadcaster`**——`fr_status` 随之停止发布（实测：之后 `/dynamic_joint_states` 无消息）
- 插件在 `on_error` 中关闭全部连接，之后不重连（`docs/fault_handling.md`："暂不做自动重连"），
  所以控制器重新上线后报的故障上层看不到，只有日志里一行 ERROR
- 插件已有自己的节点 `<name>_hw`（挂在 controller_manager 的执行器上，`reset_error` 在其上），
  生命周期与插件对象相同，硬件 `unconfigured` 时仍在

## Approach

1. **状态话题**：在 `<name>_hw` 节点上发布 `~/status`（`diagnostic_msgs/msg/DiagnosticStatus`，
   标准消息、rqt 可看，不新建消息包），QoS `transient_local`、深度 1。
   `level` OK / WARN / ERROR；`message` 为一句人话（如 `communication fault: no state frame in time`、
   `robot fault: main_code=1 sub_code=149, safety stop SI0, ...`）；`values` 含 `fault_class`、
   `main_code`、`sub_code`、`enabled`、`estop`、`safety_stop0/1`、`hardware_state`、`link`（up/down）、
   `last_frame_age_ms`、`since`（本状态开始时刻）、`previous`（上一个故障及时刻）。
   由节点上的定时器（非实时线程，1 Hz）和状态变化触发发布，**`read()`/`write()` 不发布**
2. **断线后只读重连状态流**：COMM 后后台线程每 2 s 尝试重连 **20004 状态流**（只读：不连 20003 写、
   不连 8080、不发 ServoJ），帧恢复后状态话题即反映控制器自报的状态（如 1/149）。硬件仍是
   `unconfigured`，恢复运动仍须操作者 configure → activate。`cleanup`/析构时停止该线程
3. `fault_class` 定义不变（0/1/2/3）；文档说明在本硬件上急停表现为 3 → 1，2 只在状态帧真的报
   `EmergencyStop` 的场合出现

**备选（不采用）**：只靠 `/rosout` 日志——不是接口；让 `joint_state_broadcaster` 不随硬件停用——
ros2_control 的行为，插件改不了。

**风险**：后台重连占用 20004 单客户端，驱动进程在时 `fr_probe` 连不上（与现状相同）。

## Questions

（已答：用 `diagnostic_msgs/DiagnosticStatus`；同意断线后只读重连状态流。维护者补充：
驱动看到的"断线"可能只是最先暴露的症状，实际可能是控制箱整体掉电——状态文字应写"链路中断"并把
控制器掉电（如急停）列为可能原因，不要断言是网线故障。）

实施顺序：在 [20261012-servo-drift-compensation.md](20261012-servo-drift-compensation.md) 之后。
相关代码：插件节点 `<name>_hw` 在 `on_init` 创建（`fr_robot_hardware_interface.cpp`，`reset_error` 服务所在）；
COMM 判定在 `read()`；`on_error` 交给清理线程关闭连接。本计划完成时同时改写
`fr_robot_driver/README.md`"故障与恢复"表与 `docs/fault_handling.md` 的急停描述（实测见 sdk_quirks F4、F6）。

## Result

实现（`aa52972`）：

- `status_report.hpp`：纯逻辑 `StatusTracker`（状态 → `DiagnosticStatus`，记 `since`/`previous`），单测 `test_status_report`
- 插件节点 `<name>_hw` 发布 `~/status`（`transient_local`、深度 1）；定时器 100 ms 检查，状态变化立即发、否则 1 Hz；
  `read()`/`write()` 只写原子变量（硬件状态、断线原因），不发布
- 断线后只读重连：`on_error` 的清理线程关连接后进入 `monitorLink()`，每 2 s 重连 20004；`joinCleanup()`
  （configure/cleanup/析构）唤醒并结束它。`receiver_` 改为 `shared_ptr`（状态定时器可能仍持有旧对象）
- 与计划的差别：定时器 100 ms（而非 1 Hz）以便状态变化时 `since` 准确；断线后只信任监视线程的帧，旧接收器的帧忽略
- 验证：`./scripts/check.sh` 全绿（含新增 `StatusTopic.*`：断线 → 状态流恢复 → 报 1/149，断线期间 XML-RPC 调用数、
  20003/8080 连接数、ServoJ、STOP 均不变）；真机 `192.168.58.3`（`activate_hardware:=false`，只 configure，不运动）
  `~/status` 正常：`ok`、`hardware_state=inactive`、`link=up`

真机急停演练（FR5，3.8.4.2，192.168.58.3；`activate_hardware:=false`，操作者按下约 10 s 后松开；
`~/status` 采样，日志时间为 UTC）：

| 时刻 | `~/status` |
|---|---|
| 04:24:47 | `ok`，`inactive`，`link=up`，码 0/0 |
| 04:25:06.644 | ERROR，`fault_class=3`，`unconfigured`，`link=down`，码 `unknown`，`since=…:06.644Z`，message 写"链路中断"并列出控制器掉电/急停与网络两种可能 |
| 04:25:29.144 | ERROR，`fault_class=1`，`link=up`，`main_code=1 sub_code=149`，`safety_stop0/1=1`，`enabled=0`，`hardware_state=unconfigured`，`previous` = COMM 及 `from 04:25:06.644Z to 04:25:29.144Z` |

无需任何操作者干预即从 COMM 转为 1/149（断线约 22.5 s，含控制箱重启），驱动全程不发指令。
`estop` 全程为 0，与 sdk_quirks F4 一致。小缺点：监视线程收到帧之前 `last_frame_age_ms` 为 `none`
（断线多久看 `since`）。

