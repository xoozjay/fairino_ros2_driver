# fr_client_library

法奥控制器的纯 C++17 客户端库，对标 `ur_client_library`。运行时不依赖 ROS
（ament_cmake 只负责构建、安装、测试），供 `fr_robot_driver` 的硬件插件和
`fr_probe`、`fr_servo_bench` 使用。

## 为什么不链接 libfairino

官方 SDK（`references/libfairino`，软件版本 3.8.4）每次调用新建 XML-RPC 连接且无超时、
状态结构体无锁共享、线程 detach——清单见 [docs/sdk_quirks.md](../docs/sdk_quirks.md)。
本库**不链接 libfairino**，只把 SDK 3.8.4 源码当协议规格：

- 指令的**通道、方法、参数与帧内容按 SDK 原样**（含 STOP 帧写死的计数 `44`）；
- 只做传输层健壮性改进：XML-RPC 长连接、全部 I/O 带超时、线程可 join（不 detach）；
- 状态结构体按 SDK 复制（Apache-2.0，注明出处），`static_assert` 锁定大小 960
  与关键字段偏移；
- 接口沿用 SDK 单位（deg、mm），弧度换算在插件里做。

## 三条通路

| 通路 | 端口 | 内容 |
|---|---|---|
| 状态流 | TCP 20004 | 控制器持续推送的状态帧 |
| XML-RPC | TCP 20003 | 配置与指令，HTTP/1.1 长连接 |
| 即时指令 | TCP 8080 | 少数即时指令；控制器上**驱动独占** |

`ServoSender` 自带一条 XML-RPC 连接，与配置类调用（`RobotApi`）分开，互不阻塞。

## 模块

| 头文件 | 作用 |
|---|---|
| `robot_state_384.hpp` | 状态帧结构体（SDK `ROBOT_STATE_PKG` 的 3.8.4 布局） |
| `frame_parser.hpp` | 从字节流重组状态帧：搜 `0x5A5A`、按 `data_len` 收整帧、校验和、出错重同步 |
| `state_receiver.hpp` | 接收线程，保留最新一帧（加锁）；`latest()` / 不等待的 `tryLatest()` 返回状态 + 接收时间戳 + 帧计数 |
| `spsc_queue.hpp` | 有界无锁 SPSC 队列（伺服命令：`write()` → 发送线程） |
| `tcp_socket.hpp` | 带超时的 TCP 连接，状态流与 8080 通道复用 |
| `xmlrpc_codec.hpp` | XML-RPC 值模型与编解码 |
| `http_connection.hpp` | XML-RPC 的 HTTP/1.1 长连接（poll 超时、遵守 `Connection: close`） |
| `robot_api.hpp` | 类型化 XML-RPC 调用（驱动用到的那些），参数形状按 SDK 3.8.4；不含使能、模式、负载、工具等由操作者设置的调用 |
| `instant_command.hpp` | 8080 通道（只有 STOP）：同步发送 + 按命令号等回执 |
| `servo_sender.hpp` | 非实时发送线程：SPSC 队列 + 预填 + 按固件队列深度重采样补偿时钟偏差 + 统计 |
| `fault_state.hpp` | 故障分级 `FaultClass`（`None`/`RobotFault`/`Estop`/`Comm`）与描述 |
| `robot_model.hpp` | 控制器型号字符串 ↔ `fr_description` 机型对照表（10 行） |
| `software_version.hpp` | 软件版本解析、白名单、状态帧长映射 |

## 关键设计

- **版本**：目标 3.8.4（`kTargetVersion`）。白名单只放已验证的 3.8.3（仿真器）与 3.8.4；
  第四位及以后是同一版本的 bug 修复号，解析时忽略（真机 `3.8.4.2` 即 3.8.4）。
  状态帧长由软件版本映射，**不靠帧长猜版本**；白名单之外的版本协议层大概率可用，但未验证。
- **状态**：`read()` 用 `tryLatest()`，不阻塞、不做 I/O（接收线程正写入的那一刻沿用上一帧，过期照样按通信故障处理）；
  断线由接收线程报告，重连交给上层。
- **伺服**：一条命令 = 一段 `cmdT`，按序各发一次（不跳过、不重复，落后时追赶）；
  `start()` 与空闲停发后的恢复都先预填 `prefill`（默认 3）段原地不动指令作抖动缓冲；
  `halt()` 停发并丢弃主机队列，直到下一次 `start()`。控制器拒绝（返回码）只计数，传输错误结束线程且不重试。
- **即时指令**：发指令后等回执并按命令号匹配；超时或 I/O 错误即断开连接，不自动重连。
- **参数校验**：NaN/inf 与越界参数在发送前拒绝（`InvalidArgument`），不关闭连接。

## 工具

| 可执行文件 | 用途 |
|---|---|
| `fr_probe` | 只读：打印软件版本、状态帧字段，可监视单个字段 |
| `fr_servo_bench` | 以状态帧 `mc_queue_len` 为指标的伺服性能测试（抖动、预填可配，可关闭时钟偏差补偿对照） |

`fr_servo_bench` **会让机器人动**，必须显式 `--move`；它还会检查故障状态、使能与版本白名单。
两者都独占控制器端口，驱动运行时连不上（见 `fr_robot_driver/README.md` 的"注意"）。

## 测试

`test/` 下 13 个 gtest，纯本地（假服务端、golden 报文、分帧边界、超时与断连）。
需要仿真器的用例只在设置 `FR_SIM_IP` 时运行；仿真器时序数据不可用于性能结论。

真机上尚未验证的部分（`ServoJ` 耗时与可用周期、队列行为、8080 回执取值、
`flange_cur_pos` 的单位与基坐标系等）列在 [TODO.md](../TODO.md)。
