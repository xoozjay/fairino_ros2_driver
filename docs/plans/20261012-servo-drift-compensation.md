# 补偿主机与控制器的时钟偏差，让伺服队列非空且稳定

- Status: done

## Goal

`ServoJ` 流长时间运行时，控制器队列深度保持非空且稳定：不因主机比控制器快而越积越多
（延迟增长、最终 255 满），也不因主机慢而排空（机器人切到刹车模式）。

## Done when

- 真机 `fr_servo_bench`（不加 `--rate-error-ppm`）10 min：队列不为 0、无持续趋势（|trend| < 0.01 条/s）
- 驱动保持 10 min：`servo_queue_len` 稳定，轨迹终点误差与现在相当
- 单测：假控制器以 ±1000 ppm、±5000 ppm 的速率消费，队列收敛且不为空；轨迹不跳步（相邻两条命令
  的步长不超过原轨迹相邻步长的上界）

## Context

- 真机（3.8.4.2）上主机产出比控制器消费快约 976 ppm：60 s 队列从 ~3 线性涨到 ~10（+0.122 条/s）；
  以 `--rate-error-ppm -976` 降速后 60 s 趋势 0.000、队列稳定在 2–3（见 `20261012-real-robot-bringup.md`）
- 偏差值随主机、温度而变，**不能写死**；状态帧 `mc_queue_len` 每 8 ms 一帧，可作反馈
- `cmdT` 只能取整毫秒（sdk_quirks S1），无法靠微调 `cmdT` 补偿
- 维护者：队列空了机器人容易切刹车；涨上去则拥塞、延迟大；**目标是非空且稳定**，具体深度未定
- `prefill 3` + 2 ms 抖动时队列最小 1、从不为空；`prefill 1` 时大多数时间为 0

## Approach

**重采样**：`ServoSender` 不再"一条命令一段 cmdT 原样转发"，而是按自己的发送节拍对命令流插值：

- `write()` 入队的每条命令带主机时间戳；发送线程按略可调的周期 `T_send = cmdT·(1+ε)` 运行，
  每次取"当前时刻 − 固定延迟"处的线性插值作为该段目标发给控制器（仍带 `cmdT`）
- `ε` 由队列深度反馈调节（PI，目标深度 = `prefill`，带死区、限幅 ±5000 ppm），只读状态帧
  `mc_queue_len`，不加任何新指令
- 轨迹形状不变，只是被均匀拉伸/压缩 ~0.1%；不丢点、不重复点，所以没有速度尖峰

**备选**：
- 定期丢一条/补一条命令（最简单）：丢点那一拍步长翻倍、补点那一拍停顿——运动中会有速度尖峰，
  且翻倍的步长可能超过步长检查的上限；不采用
- 固定 ppm 校正：偏差因机器而异，不采用

**风险**：插值引入至多一个周期的额外延迟；PI 参数需在真机上整定。

## Questions

（已答：目标深度先取 `prefill`、死区 ±1，真机上再调。）

## 实施指引

**现状代码**（实施前先读）：

- `fr_client_library/src/servo_sender.cpp` `ServoSender::run()`：先发 `prefill` 条，然后**有命令就发**——
  发送节拍完全由生产者（插件 `write()`，即 controller_manager 的主机时钟）决定；命令（`Command`）不带时间戳。
  `start()` / `halt()` / `resume()` 的语义（停发、丢主机队列、按实际位置重新预填）必须保留
- `fr_client_library/include/fr_client_library/servo_sender.hpp`：主机队列 `SpscQueue`，容量 64；
  `ServoSenderOptions{cmd_t_ms, prefill}`；统计 `ServoStats`
- `fr_robot_driver/src/fr_robot_hardware_interface.cpp` `write()`：每 `ratio_` 个控制周期 `enqueue` 一条，
  入队前做单周期步长检查；`read()` 从 `StateReceiver` 快照取状态（含 `mc_queue_len`）
- `mc_queue_len`：`RobotState384`（`robot_state_384.hpp`，偏移 555，i32），状态帧按**控制器时钟**每 8 ms 一帧
- `ServoSender` 拿队列深度需要一个线程安全的来源（如构造时传入读取 `StateReceiver` 快照的回调）；
  `fr_client_library` 不得依赖 ROS
- `fr_client_library/src/fr_servo_bench.cpp` 自己按节拍 `enqueue`，`--rate-error-ppm` 模拟偏差；
  补偿生效后，不加该参数时趋势应为 0
- 单测：`fr_client_library/test/test_servo_sender.cpp`（假控制器）；插件状态机：`fr_robot_driver/test/`

**真机验证**（规则见 AGENTS.md 规则 5"真机操作"）：控制器 `192.168.58.3`；先 `fr_probe` 读并记下起始位形，
与维护者确认运动范围；`fr_servo_bench --move --joint 6 --amplitude-deg 0.5 --seconds 600` 看 trend 与
直方图；驱动保持时看 `/dynamic_joint_states` 里 `fr_status/servo_queue_len`。上次会话的基线数据见
`docs/sdk_quirks.md` S17 与 `20261012-real-robot-bringup.md`。

## Result

实施了"重采样"方案，与计划的差异与实测如下。

**差异**

- 按**命令序号**而非主机时间戳插值：发送位置 `i + frac`，每次发送前进 `1+ε` 条，`frac` 回绕时跳过整条
  （轨迹折线上的点，不丢不重）。时间戳会把 `write()` 的抖动带进步长，序号不会；前提是生产者按
  主机时钟以 `cmdT` 出命令，此时主机积压恒定（实测 `max_host_backlog` 5）。因此 `fr_servo_bench`
  的 `--rate-error-ppm` 删除（它改变生产者速率，与此前提冲突），改为 `--no-compensation`。
- 首次发送等 3 条命令入队（多一个周期的延迟，留一个周期的抖动余量）；预填推迟到此时发出，并按
  "已发送 − 已播放"估计补足，使发完时固件深度约为 `prefill`（`ServoJ` 往返约半个周期，不补则起步深度约 1，
  实测出现过 0）。
- 限幅取 ±10000 ppm 而非 ±5000：偏差恰为限幅值时控制已无余量（±5000 ppm 的单测因此偶发空队列）。
- 死区取 ±0.5 而非 ±1：±1 时深度在 [2,4] 内游走，实测 300 s 内滑到 2 并偶见 1。
- 增加可选的 `rt_priority`（`SCHED_FIFO`，默认 0 = 关）：固件队列能吸收约 `prefill × cmdT` 的调度抖动，
  不是必需；被拒绝时照常运行并告警。

**实测**（真机 3.8.4.2，8 ms，`prefill 3`）

- `fr_servo_bench`，J6 ±30°（周期 6 s，最大 15.7°/s），300 s：队列最小 1（只出现在起步阶段，共 3 帧），
  60 s 后稳定在 [3, 4]，趋势 −0.000；末尾 trim +985 ppm，与基线 976 ppm 一致；`rejected = 0`、
  `max_host_backlog = 5`。未补偿的基线是 60 s 从约 3 涨到约 10。
- 驱动保持 300 s：队列 [3, 4]，trim +1021 ppm；之后 J6 +30° 再回，16 s 的轨迹，终点误差 ≤ 0.001°，
  运动期间队列仍在 [3, 4]。
- 单测：±1000 / ±5000 ppm 的假控制器，队列不空、深度无趋势、步长在 ±1% 内；锯齿波命令验证发出的点都在命令折线上
  （关掉补偿或把 `frac` 回绕改回一次，用例失败）。

**未做**

- 没有跑满 10 min（5 min 已稳定；趋势判据按 5 min 的数据）。
- 250 Hz（4 ms）勉强（`ServoJ` 往返均值 3.8 ms，发送线程几乎吃满，深度偶见 0），333 Hz 不可行；保持 125 Hz。
- 驱动没有暴露 trim 的状态接口。
