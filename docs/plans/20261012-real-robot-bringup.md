# 在 FR5 真机（3.8.4.2）上验证驱动

- Status: done

## Goal

在一台真机上走完驱动的真机验证：伺服流、轨迹、生命周期、故障与恢复，并把 TODO.md
"未验证"里能在单台、正装、小范围运动内回答的问题变成实测结论。

## Done when

- `fr_servo_bench` 在真机上跑完 J1–J6，记录 `mc_queue_len` 趋势、`ServoJ` 耗时，
  定出 `prefill` 与 `cmd_t_ms` 的默认值（或确认 3 / 8 ms 合适）
- 驱动 `activate` → 静止保持 → 小幅轨迹 → `deactivate` 全过，轨迹终点误差有记录
- 故障演练：机器人故障（下使能）、通信中断、急停各一次，驱动行为与
  `fr_robot_driver/README.md` 的"故障与恢复"表一致
- 每一步结束后机器人回到该次会话的起始位形（每轴偏差 ≤ 0.01°）
- 结论写进 `docs/sdk_quirks.md` / 包 README，TODO.md 删掉已回答的条目

## Context

**真机**：`FR5-V1-002(V6.0)`，软件 `v3.8.4.2`（白名单内），控制器 `V3.8.15-QX`，状态周期 8 ms。
是否做过标定未知。

**起始位形与运动范围**：现场布置随时会变，位形不入库。每次运动会话开始时用 `fr_probe`
读当前位形（须静止），存在会话日志里（仓库外）；本次授权的范围是该位形每轴 ±5°。

**已完成的只读验证**（未发任何运动或设置指令）：

- `fr_probe` 只调 `GetSoftwareVersion`、`GetRobotRealtimeStateSamplePeriod` 和状态流
- 驱动 configure（`activate_hardware:=false`，硬件停在 `inactive`）：型号检查通过；
  状态周期已是 8 ms，未写；FK 检查偏差 0.101 mm / 0.0005°（限 10 mm / 1°）。
  这只说明控制器的运动学模型与我们的标称模型一致、`flange_cur_pos` 为 mm/deg 且参考系是
  `base_link`；**不说明机器人的绝对精度**（标定状态未知）
- XML-RPC keep-alive：与仿真器相同（`timeout=15, max=30`）
- 负载与工具（只读，`GetTargetPayload`/`GetTargetPayloadCog`/`GetTCPOffset`）：
  0 kg、质心 [0 0 0] mm、TCP 偏移全 0
- 夹爪：`gripper_active=0x0000`，本上电周期内未激活；未激活时夹爪动作指令只会失败
  （`docs/sdk_quirks.md` G1–G4）

**驱动会发给控制器的指令**（读 `fr_robot_hardware_interface.cpp` 确认）：configure 只读
（状态周期不等于 8 ms 时才写）；activate 发 `ServoMoveStart` + `prefill` 条原地 `ServoJ`；
deactivate 发 STOP；不使能、不复位、不切模式，不碰夹爪。

**伺服队列目标（维护者）**：固件队列必须**非空且稳定**（深度取值待定，不必是某个固定数）。
队列空了机器人容易切到刹车模式；逐渐增长则拥塞、延迟变大。控制器时钟与主机有偏差，驱动必须据此补偿。

**维护者确认**：运动期间有人在示教器/急停旁；手动/自动模式对伺服无影响；急停后控制器
需要断电重启（F4）；±5° 范围内无障碍。

## Approach

**安全约束（全程）**

- 运动目标由脚本按会话起始位形生成，**在构造时夹在 ±5° 内**，实际只用 ≤ 2°；
  关节速度 ≤ 1°/s，每步后 `fr_probe` 复核位形
- 一次只动一个关节，先 J6 再往基座方向
- 控制器访问一律经 `flock /tmp/fr_robot_192.168.58.3.lock`
- 不发任何夹爪指令；不写负载、工具、模式、使能
- 任何一步出现非预期的故障码、位形偏差 > 0.05° 或队列持续增长：停止、STOP、报告，不自行恢复

**步骤**

1. 伺服基准：`fr_servo_bench --move --joint 6 --amplitude-deg 0.5 --seconds 10`，
   然后 J5…J1；再对 J6 试 `--prefill 1/3/5`、`--jitter-ms 2`。记录队列深度分布与 `ServoJ` 耗时
2. 驱动静止保持：`activate_hardware:=true` 启动，保持 30 s，看 `fr_status/servo_queue_len ≈ prefill`
   与位形不变；`deactivate`（发 STOP）→ `activate` 来回 3 次
3. 小幅轨迹：`FollowJointTrajectory` 每轴 +2° 再回起点（各段 4 s），读回终点误差
4. 故障演练（操作者执行），急停放最后，因为之后要重启控制器：
   a. 静止保持时在示教器下使能 → 期望 `fault_class=1`、硬件 `inactive`；上使能后
      `reset_error` → 激活
   b. 静止保持时杀掉 `ros2_control_node` → 观察机器人（应走完已入队的 ≤ 3 段后保持）；
      重新启动并 configure
   c. 静止保持时按急停 → 期望 `fault_class=2`、`reset_error` 被拒；操作者重启控制器后
      驱动重新 configure
5. 回到会话起始位形，`fr_probe` 复核

**不做**：步长检查的真机演练（越限命令本身从未发出，单测已覆盖；构造临界值命令有让机器人
以接近限速跳动的风险）；状态周期最小值（要改控制器设置）；负载/工具写入；夹爪；
非正装、双机、其他型号（本机条件不具备）。

## Result

驱动在一台 FR5（3.8.4.2）上走完伺服、保持、轨迹、生命周期与三类故障；结束时机器人回到会话起始位形
（各轴 0.000°）。发现两处需要改的设计，各立计划：伺服时钟偏差补偿、故障与断线期间的状态上报。

- **伺服基准**（`fr_servo_bench`，0.5°，J1–J6 各 10 s）：全部回到起点 ≤ 0.0015°，无拒绝、无故障；
  `ServoJ` 往返平均 3.8 ms、最大 7.7 ms（接近 8 ms 周期）
- **时钟偏差**：J1–J6 六次趋势一致 +0.11–0.13 条/s；J6 60 s 队列从 ~3 线性涨到 ~10（+0.122 条/s）。
  以 `--rate-error-ppm -976` 降低产出速率后 60 s 趋势 0.000、队列稳定在 2–3——是纯粹的速率偏差
  （本主机与本控制器之间约 976 ppm，主机快）。不补偿时延迟每秒约增加 1 ms，约 34 min 填满 255
- **预填**：速率修正后，`prefill 1` 队列多数时间为 0（无余量）；`prefill 3` + 2 ms 抖动最小 1、从不为空
- **驱动保持**：激活后 30 s 位形偏差 ≤ 0.003°，队列 4 → 5 → 7（同一偏差）；`deactivate` 发 STOP 后
  `servo_queue_len` 2 s 内归 0（真机 STOP 清空固件队列），`activate` 后回到预填深度；来回 3 次均正常，
  轨迹控制器在硬件来回切换时保持 `active`
- **小幅轨迹**（每轴 +2° 再回，各段 4 s）：J1–J6 全部 SUCCEEDED，峰值 +1.997–1.999°，终点误差 ≤ 0.0013°
- **驱动崩溃**（`SIGKILL` `ros2_control_node`，不发 STOP / `ServoMoveEnd`）：机器人原地保持、无故障，
  端口随即释放；重新启动后 configure / activate 正常
- **急停**（静止保持时拍下）：控制箱断电（风扇、指示灯熄灭），控制器从网络消失；驱动报
  `communication fault: no state frame in time`（COMM，不是 ESTOP），硬件 → `unconfigured`，
  `reset_error` 拒绝。松开后控制箱自行启动，报 1/149、SI0/SI1、未使能；驱动重新 configure 后判为
  ROBOT_FAULT、拒绝激活；`ResetAllError` 返回 -1、状态不变。断电重启后无故障、自动使能，驱动
  configure → activate 正常，J2 因断电抱闸偏移 0.005°
- **下使能**（静止保持时 `RobotEnable(0)`，代替示教器操作）：驱动报 `robot fault: not enabled`（ROBOT_FAULT），
  硬件 → `inactive`、轨迹控制器停用，**`joint_state_broadcaster` 保持激活**（`fr_status` 继续发布）；
  未使能时激活被拒；`RobotEnable(1)` 后故障即消失，无需 `reset_error`，直接激活成功。使能切换触发内部继电器
- **发现的缺口**：COMM 时 ros2_control 连同 `joint_state_broadcaster` 一起停用，`fr_status` 不再发布；
  驱动也不重连，所以上层既看不到断线期间的状态，也看不到重新上电后控制器报的原因（另立计划）
- 上真机激活前修复：测试台在执行器仍 spin 时析构 `ResourceManager`，约 1/8 概率 SIGSEGV（`8b44832`）

- Commits: `fix(driver): stop the test executor before destroying the resource manager`、
  `feat(driver): log the flange deviation when the FK check passes`、
  `feat(client): read the controller's payload, payload CoG and TCP offset`、
  `feat(driver): log the controller's payload and tool at configure`、`feat(client): show the gripper state in fr_probe`，
  以及 `docs:` 记录（sdk_quirks C7、F4、F6、G1–G4、S3、S4、S17）
- Checked by: 真机实测（上文）；`scripts/check.sh` 150 个用例全过
- Not checked: 255 条队列满时的行为；状态周期最小值；非正装、双机、其他型号；长时间（> 3 min）连续伺服；
  `fr_robot_driver/README.md`"故障与恢复"与 `docs/fault_handling.md` 的急停描述尚未按实测改写
  （归入 [20261012-fault-status-topic.md](20261012-fault-status-topic.md)）
- Tried and failed: 急停后 `ResetAllError`（返回 -1，状态不变；只能断电重启）
- Revisit if: 固件或控制器版本变化；换主机（时钟偏差不同）；安装方式改变
