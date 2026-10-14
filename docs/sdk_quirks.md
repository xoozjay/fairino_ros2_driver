# libfairino SDK 陷阱清单（固件 3.8.4）

> 写驱动前必读。官方 C++ SDK 质量很差，本文记录所有会影响 ros2_control 驱动
> 正确性/实时性的行为，避免在硬件插件里踩坑。
>
> - **基线**：SDK `v2.2.4-robot3.8.4`（commit `831c662`，`libfairino.so.2.2.4`），
>   对应控制器固件 **v3.8.4**。行号均指该 tag 下的 `libfairino/src/src/Robot/robot.cpp`
>   （`references/fairino-cpp-sdk` 或 `references/libfairino` 中 `git show 3.8.4:...`）。
> - **来源标记**：🔍 源码确认　📋 维护者实测/经验　❓ 待真机验证
> - **仿真器**：本机 `192.168.58.2`（官方 SimMachine Docker 镜像 v3.8.3），
>   只能做**功能/协议**验证；它与客户端同机运行，**延迟、时序数据无参考价值**，须在真机测。
>   没有 3.8.4 的 SimMachine，以 3.8.3 作替身：SDK 3.8.3 → 3.8.4 状态帧布局完全相同；XML-RPC
>   仅新增 8 个方法（Focus*/固件升级/`SetRobotType`）、1 个改参数（`ExtAxisMoveJ`），
>   **驱动用到的方法全部未变**。仿真器上通过 ≠ 3.8.4 真机通过，上线前仍须真机验证。
> - 许可：SDK 为 Apache-2.0，可移植进本仓库（AGPL-3.0）。

---

## 一、版本

| # | 陷阱 | 驱动对策 |
|---|---|---|
| V1 📋 | **SDK 版本必须与固件版本严格一致**。`references/` 下的 SDK 与文档仓库已锁定到 3.8.4（见 AGENTS.md 版本约束）；**3.9.x 的 SDK 连 3.8.4 固件是错的**（状态端口 20005、结构体不同） | 一律以 `v2.2.4-robot3.8.4` 为准；升级固件 = 重新审计本文 |
| V2 🔍 | 版本不匹配**检测很弱**：`FRTcpClient::RecvPkg` 先按 `sizeof(ROBOT_STATE_PKG)`（**960 字节**，`pack(1)`）读，帧头长度更长时**静默接受、只用前缀**；更短时返回 `-3` 且已多读了下一帧的字节（流错位） | `on_configure` 用 `GetSoftwareVersion` 校验控制器版本，不符直接失败；不要依赖 SDK 的 `-3` |
| V3 🔍 | 接收缓冲固定 1024 字节（栈上 `allRecvBuf[1024]`），帧长 > 1024 时**栈溢出**——固件 ≥ 3.8.6 的帧（1184 B 起）即触发：**控制器升级而 SDK 未升级 = 内存破坏** | 自研接收器按帧头长度动态处理 |
| V4 🔍 | `GetSDKVersion()` **不访问控制器**，只返回编译期常量 `"SDK V2.2.4.0-robot v3.8.4"` | 不能用它判断固件版本 |

### 驱动所需字段与方法（兼容下限 3.7.6）🔍

**状态帧**：驱动只需以下 29 个字段，在 SDK 3.7.6 → 3.9.8 全部 20 个 tag 中**偏移与大小逐一相同**，
且全部位于前 **775 字节**内（3.7.6 的帧长 888）；各版本注释含义一致，3.9.8 注释补全了单位。

| 用途 | 字段（偏移） |
|---|---|
| 帧头 | `frame_head`(0) `frame_cnt`(2) `data_len`(3) |
| 关节 | `jt_cur_pos`(16) deg、`actual_qd`(160) deg/s、`jt_cur_tor`(384) N·m |
| 故障/状态 | `program_state`(5) `robot_state`(6) `main_code`(7) `sub_code`(11) `robot_mode`(15) `EmergencyStop`(549) `motion_done`(550) `collisionState`(559) `safety_stop0/1_state`(564/565) `rbtEnableState`(771) |
| 位姿 | `tl_cur_pos`(64) `flange_cur_pos`(112)：mm + 绕固定轴 X/Y/Z 的 deg；`tool`(432) `user`(436) |
| IO | `cl_dgt_output_h/l`(440/441) `tl_dgt_output_l`(442) `cl_dgt_input_h/l`(443/444) `tl_dgt_input_l`(445) `cl_analog_input`(446) `tl_anglog_input`(450) |

❓ SDK 3.9.5 起改用 20005（CNDE），≥ 3.9.5 固件是否仍提供 20004 未知。

**XML-RPC / 8080**：驱动用到的 20 个方法在 3.7.6 → 3.8.4 间只有 3 处差异：

| 方法 | 差异 |
|---|---|
| `SetLoadWeight` | 3.7.6–3.7.7 为 `(weight)`；3.7.8 起 `(loadNum, weight)` |
| `ServoJ` | 3.8.3 起末尾追加 `id`（7 → 8 个参数） |
| `StopMotion` | 3.7.6–3.8.2 走 XML-RPC `StopMotion`；3.8.3 起改走 8080 `/f/b` STOP 帧（3.8.3 仿真器上 XML-RPC 方法仍存在）。❓ 3.8.3 之前的固件是否支持 8080 STOP |

其余（`GetSoftwareVersion`、`GetSlaveFirmVersion`、状态周期读写、`ServoMoveStart/End`、`RobotEnable`、
`ResetAllError`、`Mode`、`SetSpeed`、`DragTeachSwitch`、`SetLoadCoord`、`SetToolCoord`、`SetDO`、`SetToolDO`、
8080 的 PAUSE/RESUME）3.7.6 → 3.8.4 完全一致。仿真器实测：无参 getter 多传参数会被**忽略**；
❓ 有参方法少传/多传参数（如 7 参数 `ServoJ`）的行为待测。

### 版本识别

**① XML-RPC `GetSoftwareVersion`（权威）** 🔍：无参数，返回
`[errcode, robotModel, webVersion, controllerVersion]`。无需 SDK 和 `RPC()` 握手，
curl 即可查询（XmlRpc++ 默认路径 `/RPC2`）：

```bash
curl -s -H 'Content-Type: text/xml' http://192.168.58.2:20003/RPC2 --data \
  '<?xml version="1.0"?><methodCall><methodName>GetSoftwareVersion</methodName><params></params></methodCall>'
```

📋 本地仿真器（官方 SimMachine v3.8.3）返回
`[0, 'FR5-V1-002(V6.0)', 'v3.8.3', 'V3.8.8-LA']`，而其状态帧为 960 B（与 SDK 3.8.4 布局一致）——
**软件版本是第二个字符串 `webVersion`**（`v3.8.3`，与仿真器发行版本一致）；`controllerVersion`（`V3.8.8-LA`）
是厂商的另一套编号，只显示、不比较。（厂商的 `GetFirmwareVersion` 指控制箱板/驱动器固件，别和它混淆。）
白名单与状态帧长都按软件版本。📋 维护者确认真机软件版本为 **3.8.4.2**（第四位及以后是 bug 修复版本，不改功能，
视为同一版本，解析时忽略）；仿真器只有 3.8.3，没有 3.8.4。❓ 真机 `GetSoftwareVersion` 返回的确切字符串形式待首次联机确认。另有 `GetSlaveFirmVersion` / `GetSlaveHardVersion`
（控制箱板、6 个驱动器、末端板，各 8 个字符串）；注意 SDK 的 `GetFirmwareVersion` 在安全停止时会被
`GetSafetyCode()` 拦截返回 99。❓ 控制器是否开放 `system.listMethods` 内省待测。

**② 20004 状态帧长度（由软件版本映射，不用来猜版本）** 🔍：帧格式 `5A 5A | frame_cnt(1) | data_len(2, 小端) | ... | checksum(2)`，
帧长 = `data_len + 7`。各 SDK tag 的 `sizeof(ROBOT_STATE_PKG)`（`pack(1)` 实算）：

| 软件版本 | 帧长 | data_len |
|---|---:|---:|
| 3.7.6 | 888 | 881 |
| 3.7.8 ~ 3.8.1 | 902 | 895 |
| 3.8.2 | 954 | 947 |
| **3.8.3 ~ 3.8.5** | **960** | **953**（`0x03B9`） |
| 3.8.6 | 1184 | 1177 |
| 3.8.7 ~ 3.9.4 | 1236 | 1229 |
| 3.9.5 | 2073 | 2066（且端口改为 20005） |
| 3.9.6 ~ 3.9.7 / 3.9.8 | 2074 / 2075 | 2067 / 2068 |

> 此表在代码中的副本：`fr_client_library/software_version.hpp` 的 `kStateFrameProfiles`，修改时两处同步。
> 表中版本区间之外（如 3.7.7：无 SDK tag）不猜帧长，驱动拒绝启动。

3.8.3/3.8.4/3.8.5 字段布局逐项一致，**帧长只能确认"布局兼容"，不能区分具体版本**；
新版本字段均追加在 `check_sum` 之前。快速查看：`nc <ip> 20004 | head -c 5 | xxd`，
期望 `5a5a ?? b903`。

### 跨版本兼容性（SDK 各 tag 静态比对，非抓包）

**状态帧（20004）：严格追加** 🔍

- 3.7.6 → 3.9.8 每次变更都是在 `check_sum` 前**追加**字段，从未插入/删除/改类型；
  嵌套结构体（`robot_aux_state` 25 B、`EXT_AXIS_STATUS` 29 B、`WELDING_BREAKOFF_STATE` 2 B、
  `RobotTime` 9 B）自 3.7.8 起大小不变；**3.8.4 的全部 70 个字段在 3.9.8 头文件中偏移逐一相同**
- 推论：按 `data_len` 收整帧、校验和覆盖整帧、只解析已知前缀 → **新固件可读**（前向兼容）；
  字段 `offset+size <= 帧长` 才可用 → **旧固件可读**（后向兼容）
- SDK 自身做不到：固定 1024 缓冲（V3）、长度不等即 `-3`（V2）——"必须严格匹配"是 **SDK 的缺陷，不全是协议的**
- ❓ 3.9.5+ SDK 改走 20005 CNDE（按名订阅、紧凑拼接，类似 UR RTDE），**新固件是否仍提供 20004 旧格式未知**
- ❓ 布局不变不代表语义不变（单位、含义），头文件无法证明

**XML-RPC（20003）：方法集基本只增，但参数不是严格追加** 🔍

| 3.8.4 对比 | 删除 | 新增 | 参数形状变化 |
|---|---:|---:|---:|
| 3.8.7 | 0 | 50 | 8 |
| 3.9.8 | 3 | 128 | 23 |

- 多数是尾部追加参数（`ServoJ` 在 3.8.x 追加 `id`；`ServoJT`、`SetLoadCoord` 追加）
- **也有中间插入**：3.9.x `ServoCart` 在第 3 位插入外部轴数组，`acc` 之后全部错位——位置参数协议下这是破坏性变更
- ❓ 控制器对参数个数不符的处理未知（`ERR_PARAM_NUM = 3` 的存在暗示严格校验）；
  可在 3.8.4 上用 7 参数（3.7.8 形状）调用 `ServoJ` 验证
- 驱动用到的方法（`ServoMoveStart/End`、`ServoJ`、`RobotEnable`、`ResetAllError`、`Mode`、`SetSpeed`、
  `SetDO`、`DragTeachSwitch`、`SetLoadWeight`、`GetSoftwareVersion`、`SetRobotRealtimeStateSamplePeriod`）
  3.8.4 → 3.9.8 线上形状**均未变**，仅 `SetLoadCoord` 追加一个参数（3.9.4+ 的 `comType` 只是 SDK 侧选 UDP，不改 XML-RPC 报文）

**③ XML-RPC 方法集合（最精确）** 📋：服务端（xmlrpc-c）开放标准自省接口，`system.listMethods`
返回全部方法名，`system.methodExist(name)` 检查单个方法——**只读，不调用任何业务方法**。
仿真器返回 411 个方法；与各版本 SDK 实际调用的方法名对照：

| SDK | 仿真器缺失的方法数 |
|---|---:|
| 3.8.0 / 3.8.2 / 3.8.3 | 2（`AuxServosetStatusID`、`SetCtrlOpenLuaErrCode`，前者疑为 SDK 拼写） |
| **3.8.4** | **10**（3.8.4 新增的 8 个全部缺失：`SetRobotType`、`Focus*`、固件升级类） |
| 3.8.5 | 24 |

- 方法集合能区分 **3.8.3 与 3.8.4**，状态帧长度做不到（两者都是 960 B）；结论：仿真器是 3.8.3
- 与之吻合的是 `webVersion`（`v3.8.3`），`controllerVersion`（`V3.8.8-LA`）是另一套编号；
  ❓ 真机上 `webVersion` 是否总等于 SDK 发布版本号待确认
- 自省接口还暴露 `system.shutdown`（**驱动绝不调用**）与 `system.multicall`（SDK 未使用，本项目不用）

**驱动策略**：

1. `on_configure` 用 ① 取软件版本（`webVersion`），解析失败或不在**真机验证白名单**内即拒绝启动
   （当前登记 3.8.3 仿真器、3.8.4 目标；3.8.4 真机待验证）；
2. 用 ③ `system.methodExist` 确认驱动用到的每个方法都存在（能力检查，比版本号更直接）；
3. 状态接收器按软件版本取帧长（`stateFrameSize()`），只接受该版本的帧；更长的帧只解析 3.8.4 前缀，
   更短的缺失尾部清零。无已知帧长的版本（如 3.7.7）拒绝启动。

任一不符即拒绝启动；新版本须真机验证后才加入白名单。白名单之外的版本协议层大概率可用，只是未经验证。

## 二、命令通道（XML-RPC，TCP 20003）

| # | 陷阱 | 驱动对策 |
|---|---|---|
| C1 🔍📋 | **每次 API 调用都新建 `XmlRpcClient`，调用完立即 close**（全文 370 处 `XmlRpcClient c(serverUrl, 20003)`）——一次调用 = TCP 握手 + HTTP POST + 关闭。**`ServoJ` 也是如此** | 热路径（`write()`）绝不直接调用 SDK；见第六节 |
| C2 🔍 | 短连接累积 `TIME_WAIT`：125 Hz × 60 s ≈ 7500 个；1 kHz 时 ≈ 60000，**耗尽临时端口** | 用长连接（见 C7） |
| C3 🔍 | **无超时**：`XmlRpcClient::execute` 调 `_disp.work(-1.0)`，控制器卡住/断网时**无限阻塞** | 所有调用放非实时线程，并有外部看门狗 |
| C4 🔍 | 每个 API 开头 `IsSockError()`：重连期间 `while (GetReConnState()) Sleep(100)` 自旋——**任意调用可阻塞到重连超时（默认 30 s）** | 同上 |
| C5 🔍 | `g_sock_com_err` **粘滞**：任一线程出错（含看门狗）置位后，所有 API 返回错误，直到 `CloseRPC()`+`RPC()` | 错误恢复 = 整体重连流程，映射到 lifecycle `on_error` |
| C6 🔍 | SDK **客户端**内置的是 **XmlRpc++ 0.7**（Chris Morley，LGPL）；服务端才是 xmlrpc-c（见 C7）；绝大多数方法只是"参数数组 → 返回 int"的薄封装 | 自研时 `robot.cpp` 可直接当**协议规格书**读（方法名 + 参数顺序） |
| C7 📋 | **服务端支持 HTTP keep-alive**：仿真器响应头 `Server: Xmlrpc-c_Abyss/1.54.5`、`Connection: Keep-Alive`、`Keep-Alive: timeout=15, max=30`——**每条连接最多 30 个请求，空闲 15 s 由服务端关闭**；SDK 不用纯属 SDK 的问题。真机（软件 3.8.4.2、控制器 V3.8.15-QX）响应头相同；仿真器在本机运行，延迟数据无参考价值 | 自研客户端用长连接；满 30 个请求或空闲接近 15 s 时，在两次请求之间换一条新连接（不重发可能已送达的请求）。伺服流约每 240 ms 有一次调用含 TCP 握手（亚毫秒级），由固件队列吸收 |
| C9 📋 | **keep-alive 实测（仿真器）**：`Keep-Alive: max=30` 是**静态值，不递减**；第 30 个请求的响应头变为 `Connection: close`（且无 `Keep-Alive` 头），随后服务端关闭连接；空闲 15 s 后服务端关闭，在陈旧连接上发送会成功但读到 EOF（无法判断请求是否已被处理）。响应头名为 `Content-length`（小写 l） | 只在两次请求**之间**轮换：遵守 `Connection: close`，空闲接近超时（留 2 s 余量）时先换新连接；请求中途出错 → 关闭并上报，**不重试**（可能已送达）。已在仿真器上连续 70 次请求验证 |
| C8 ❓ | 服务端处理单次 XML-RPC 调用的开销未知——长连接只省掉握手，不省服务端处理 | 伺服周期须在真机实测后确定 |

## 三、伺服（ServoJ）

| # | 陷阱 | 驱动对策 |
|---|---|---|
| S1 📋 | **固件内部以 1 kHz 运行，只在相邻 `ServoJ` 点之间做线性插值，不做更复杂的平滑**（维护者确认）：每条 `ServoJ` 是一段时长 `cmdT` 的直线段，速度在段与段之间是阶跃的。`cmdT` 应为 1 ms 的整数倍，**否则向上取整**（1.5 ms → 2 ms） | `cmdT` 只用整毫秒；`update_rate` 必须整除 1000（1000/500/250/200/125/100…）；**目标序列本身必须平滑**（来自轨迹控制器），固件不会替你平滑 |
| S2 🔍 | 头文件注释"建议范围 [0.001~0.0016]"疑为笔误；SDK 自带示例全部用 `cmdT = 0.008`（125 Hz） | 起步 125 Hz，实测后再调 |
| S3 📋 | `cmdT` 与实际下发周期不一致：真机上主机节拍与控制器消费存在约 1000 ppm 的速率偏差，队列随之线性增长（S17）。推论：每条是一段 `cmdT` 的直线段、按 1 kHz 逐段消费（S1、S15），发送周期比 `cmdT` 长 → 队列排空、段间保持（卡顿），短 → 队列累积（S16） | 保证 `cmdT == 1/update_rate`；频率偏差由 `ServoSender` 按 `mc_queue_len` 反馈重采样补偿（S15），不靠改 `cmdT`（只能取整毫秒，S1） |
| S4 📋❓ | 伺服流中断（不再发 ServoJ）：仿真器上机器人把已入队的命令走完后保持，不报故障（S12、S16）；真机（3.8.4.2）：驱动进程被 `SIGKILL`（未发 STOP / `ServoMoveEnd`）后机器人原地保持、无故障，端口随即释放，重启驱动可正常 configure / activate | 驱动侧仍必须有发送超时看门狗；"停发"不等于"停止"（S15） |
| S5 🔍 | 3.8.4 的 `Servo*` **没有 `comType` 参数**，只能走 XML-RPC；UDP 20007 伺服通道 3.9.4 才有 | 不可用，别抄新版代码 |
| S6 🔍📋 | SDK 示例把 `ServoJ` 包在 `ServoMoveStart()`/`ServoMoveEnd()` 之间，但 **`ServoMoveStart`/`ServoMoveEnd` 在 3.8.x 固件层面都是空语义指令**（维护者确认）——**不要依赖这一实现细节**，仍按 API 约定成对调用；`axisPos` **会被解引用，不可传 nullptr**（传全零） | 不要把"已调用 ServoMoveEnd"当作机器人已停止的依据 |
| S7 🔍 | 单位是**度**（关节）、mm（外部轴）；ROS 是弧度 | 在插件边界统一换算 |
| S8 🔍 | 每次 `ServoJ`/`ServoMoveStart` 先查 `GetSafetyCode()`：SI0/SI1 安全停止时返回 **99**——判断依据是可能撕裂/过期的状态包（见 R2） | 99 → 状态接口报安全停止 + `ERROR` |
| S9 🔍 | 3.8.4 已有力矩伺服 `ServoJTStart/ServoJT/ServoJTEnd`（`interval` 1–8 ms，Nm） | 将来 effort 命令模式可用 |
| S10 📋 | **仿真器上伺服实测**（`ServoSender`，8 ms 周期；时序无效，只说明功能行为）：以 0.1°/周期的斜坡推进关节 1 共 8°，状态帧回读 7.99979°，推回后 0°，0 次拒绝，`ServoMoveEnd` 返回 0。**无 `ServoMoveStart` 直接发 `ServoJ` 或 `ServoMoveEnd` 也返回 0**（印证 S6：两者是空语义） | 仍按 API 约定成对调用 |
| S11 📋 | **目标单步跳变过大会使控制器进入故障**：单次跳变 1° / 2° / 4° 被跟随，**8°（及 30°、6°）在约 14 帧后触发故障** `main_code=1, sub_code=23`（含义 ❓），此后**每一帧 `ServoJ` 都被拒绝，返回码 14**（印证 F2），直到 `ResetAllError`；复位后 `main_code=0`，使能保持（印证 F5）。阈值在仿真器上测得，真机 ❓。速度 = Δq/`cmdT`：8°/8 ms = 1000°/s 触发故障、4°/8 ms = 500°/s 通过，与"固件最大关节角速度保护"（S14）吻合（推论） | 插件的单周期步长检查（fault_handling §六）不是可选项；故障后恢复运动时目标必须从**实际位置**起用斜坡，不能一步跳回（实测：恢复时一步跳回 6° 再次触发故障） |
| S12 📋 | **伺服流中途停发（连接仍在、未调 `ServoMoveEnd`）**：机器人把已入队的命令走完（5.52° → 5.9998°，最后目标 6.0°；见 S15/S16）后保持，**不报故障**；`robot_state` 由 2 变回 1。仿真器 ❓ 真机 | 停发 = 保持最后目标，不是自动停止或回零；需要真正停止时用 STOP（8080）或下使能，不要靠"不发了" |
| S13 🔍 | `ServoJ` 的 `acc/vel/filterT/gain` 在 SDK 中"暂不开放，默认为 0"，`axisPos` 不可省略；SDK 示例全部传 0、`id=0`、`cmdT=0.008` | `RobotApi::servoJ` 固定发 0 与四个零外部轴 |
| S14 📋 | **`ServoJ` 直接控制机械臂，不受全局速度（`SetSpeed`）与加速度限制**，仅有固件最基本的保护：**最大关节角速度、最大加速度（电流）与关节限位**（维护者确认）。越过时控制器进入故障（S11） | `SetSpeed` 对伺服无效，**不能当作安全手段**；速度/加速度/限位保护全在 ROS 侧：URDF 限位 + `enforce_command_limits` + 插件步长检查（fault_handling §六）。速度 = Δq/`cmdT` |
| S15 📋 | **固件内部有 255 条的缓冲区，存放 `ServoJ` 等运行指令**（维护者确认；S16 实测印证） | 255 × `cmdT` 的运动可以已在固件里排队（8 ms 时 ≈ 2.04 s）：**停发不等于停止**，已入队的仍会执行；要立即停下用 `STOP`（8080，S16）。**这个队列就是抖动缓冲**：启动/恢复时预填 `prefill` 段原地不动的指令，代价是常量延迟 `prefill × cmdT`。抖动本身不会让队列变深（发送按绝对时间表，不累积）；**会累积的是主机与控制器的时钟频率偏差**（S17）。`ServoSender` 因此不逐条转发，而是按自己的节拍（周期 `cmdT·(1+ε)`）对命令折线线性插值，每段仍带 `cmdT`：不丢点、不重复点（"只发最新"或丢/补一条会使某段速度为 0 或翻倍，而固件限制加速度，S14），轨迹只被均匀拉伸 ε；ε 由 PI 按 `mc_queue_len` 调节（目标深度 `prefill`、死区 ±1、限幅 ±10000 ppm（1%））。代价：首次发送要等 3 条命令入队（多一个周期的延迟），流在 ε≠0 时结束则最后一条命令不会被精确送达 |
| S16 📋 | **队列行为实测（仿真器）**：① 100 条 `ServoJ` 在 20 ms 内全部被接受，机器人按 `cmdT` 逐条执行（约 800 ms），状态帧 **`mc_queue_len`（偏移 555，i32）跟随占用量**（约 90 → 0），发送早已停止、位置仍走到最后一条；② 约 255 条以后**每次 `ServoJ` 调用阻塞到有空位**：单次耗时 #1–200 约 0.2 ms，#301–600 恒为 8.00 ms（= `cmdT`），**不拒绝、不丢弃**，600/600 全部执行；③ `ServoSender` 按名义节拍发送 4 s：`mc_queue_len` 稳定在 0–1，无累积；④ 队列中途发 `STOP`：约 100 ms 内减速停下，`mc_queue_len` 归零，无故障，之后伺服可重新启动；⑤ `fr_servo_bench`（FIFO 发送，6 s，8 ms）：无预填时队列贴着 0–1（零余量），**预填 4、发送抖动 0–4 ms 时稳定在 2–4（均值 2.96）**，抖动不改变均值；发送比消费快 2% 时队列以 **+2.535 条/s** 上升（理论 2.55，约 96 s 到 255），与抖动无关。真机见 S17；**队列满时真机同样让服务端等待、XML-RPC 调用阻塞到有空位**（维护者确认；实测：原地不动的 `ServoJ` 以快 10% 的速率发 30 s，`mc_queue_len` 到 255 并保持约 7 s，调用平均 3.8 → 5.2 ms、最大 10.2 ms，0 拒绝、无故障、机器人不动） | 把 `mc_queue_len` 作为伺服健康指标：稳态应约 0–2，持续增长说明发送快于消费（报警/降速）。**分辨率受状态周期限制**（仿真器最小 8 ms，`SetRobotRealtimeStateSamplePeriod` 设 ≤ 7 返回码 4），固件以 1 kHz 消费，所以只能看深度的分布与趋势（min/mean/max、斜率），看不到 8 ms 以内的欠载；队列满时调用会阻塞 `cmdT`，所以发送线程的请求超时不会误判，但延迟已经很大；急停用 `STOP`，不要靠停发 |
| S17 📋 | **真机伺服实测**（3.8.4.2，8 ms，`fr_servo_bench` 与驱动）：`ServoJ` 往返平均 3.8 ms、最大 7.7 ms（接近 8 ms 周期）；主机产出比控制器消费**快约 976 ppm**（J1–J6 六次趋势一致 +0.11–0.13 条/s，60 s 从 ~3 涨到 ~10；以 `--rate-error-ppm -976` 降速后趋势 0.000），即不补偿时延迟每秒约增 1 ms、约 34 min 填满 255；`prefill 3` + 2 ms 抖动队列最小 1，`prefill 1` 大多数时间为 0；`STOP` 后 `mc_queue_len` 2 s 内归零 | 偏差因主机、温度而异，须按 `mc_queue_len` 反馈补偿（不能写死；`ServoSender` 已实现，补偿后 300 s 趋势 0、队列 [3,4]，trim 稳定在 +985…+1091 ppm）；维护者：**运动中**队列须非空且稳定——发送出现空隙（欠载）时某一时刻目标不变，关节驱动器倾向于把电机停住，欠载期间加速度不断变化、发出声响（不是刹车） |
| S18 📋 | **空闲时停发再恢复**（3.8.4.2，原地不动的 `ServoJ` 流，不调 `ServoMoveEnd`）：停发后队列排空、`robot_state` 2 → 1，停 5 s 期间位置偏差 ≤ 0.0004°、无故障、使能保持；之后先补 3 段原地不动的再续发 3 s，偏差 ≤ 0.002°、关节速度 ≤ 0.09 °/s、队列 0–1、无故障 | 机械臂静止时可以停发；恢复时先预填原地段再接运动，运动中不能留空隙（S17） |
| S19 📋 | **伺服跟随滞后不随速度发散**（3.8.4.2，FR5，J6 升余弦往返、周期 0.5 s，`fr_servo_bench`，`cmdT = 8 ms`，`prefill 3`）：峰值 90、135、180°/s 时命令与反馈的互相关滞后为 133、121、122 ms，最大误差 10.5°、14.8°、19.6°（0.183、0.258、0.342 rad），与纯延迟模型（摆幅 A、周期 T、滞后 τ：误差幅值 A·sin(πτ/T)）相符；经驱动与 JTC 在 ≤ 90°/s 下测得 116–121 ms。对照：`prefill 1`（135°/s）滞后 88 ms，少 33 ms，但队列均值只从 1.93 降到 0.21（约少 14 ms），差值未解释；其余组成（主机端 `kLead`、控制器内部插补与总线传输、伺服驱动器与机械响应、状态帧采样）未单独测量 | 跟随误差约为速度 × 滞后，路径容差按滞后上限 133 ms 计（最快关节 3.2 rad/s 时约 0.43 rad，取两倍）；不要靠降低 `prefill` 减少滞后：`prefill 1` 时运动中队列大部分时间为空，违反 S17 |

## 四、状态通道（TCP 20004）

| # | 陷阱 | 驱动对策 |
|---|---|---|
| R1 🔍 | 控制器**推送全量** `ROBOT_STATE_PKG`（960 B），周期由 `SetRobotRealtimeStateSamplePeriod(ms)` 设定（示例用 10）；仿真器默认 **8 ms**（`GetRobotRealtimeStateSamplePeriod` 读回与帧间隔一致）；❓ 最小值与真机默认值待测 | 启动时显式设置并回读 `GetRobotRealtimeStateSamplePeriod` |
| R2 🔍 | **无锁**：接收线程 `memcpy` 写入，`GetRobotRealTimeState` 同时 `memcpy` 读出 → **撕裂读** | 自研接收器用 seqlock / `realtime_tools::RealtimeBuffer`；或至少读两次比较 `frame_cnt` |
| R3 🔍 | **出错时 SDK 把状态包 `memset` 成 0**（`CloseRPC`、指令线程出错、看门狗）；`RecvPkg` 帧头非 `0x5A5A` 时**返回 0 且缓冲为全零**，状态线程照样拷贝 → **关节位置读成全 0 却像有效数据** | 每次读状态都校验 `frame_head == 0x5A5A` 和 `frame_cnt` 在变化；**绝不用全零状态初始化命令**（否则激活瞬间机械臂冲向零位） |
| R4 🔍 | 帧同步不会找 `0x5A5A` 重新对齐：一旦错位就持续出错 | 自研接收器按帧头搜索重同步 |
| R5 🔍📋 | `frame_cnt` 是 `uint8_t`（256 回绕）；仿真器上**每帧递增 2**（5, 7, 9…） | 判断"新帧"用 `!=`，不要假设 +1，也不要比大小 |
| R6 🔍 | 单位：`jt_cur_pos` 度，`jt_cur_tor` Nm；❓ `actual_qd` 推测为 deg/s 待确认 | 同 S7 |
| R7 📋 | **`flange_cur_pos` 的约定（仿真器 6 个位形实测）**：mm 与 deg，在 URDF `base_link` 坐标系下，姿态 `R = Rz(rz)·Ry(ry)·Rx(rx)`，位置 = `fr_description` 的 FK（`wrist_3` 沿 z 偏 `flange_offset`）；与 FK 的差 ≤ 0.25 mm / 0.1°。`tl_cur_pos`（TCP）在未配置工具时与它相同 ❓真机 | 用于型号/零位一致性检查（fault_handling §八）；比较姿态用旋转矩阵，不用欧拉角 |
| R8 📋 | **型号字符串**：`GetSoftwareVersion` 的第一个字符串，如 `FR5-V1-002(V6.0)`；完整型号表见官方手册 `CobotsManual/system.rst`（FR3/5/10/16/20/30 的 V5.0 与 V6.0 代、FR3-WML/WMS、FR5WM、FR3MT 等）。**只有 FR5 V6.0 在仿真器上验证过** | `robot_model.hpp` 映射到 `fr_description` 机型；V5.0 及更早、Mirror 等无对应运动学，视为未知 |

## 五、生命周期与线程

| # | 陷阱 | 驱动对策 |
|---|---|---|
| L1 🔍 | `RPC()` 启动 **4 个 detach 线程**（状态、指令发送、指令接收、看门狗），全部捕获 `this`，并 `Sleep(2000)+Sleep(1000)` —— **成功也要阻塞 ≥ 3 s**（`robot.cpp:275`） | 只在 `on_configure` 调用 |
| L2 🔍 | `RPC()` 失败返回时，已启动的线程**不会被停止** | 失败后同样走 `CloseRPC()` |
| L3 🔍 | `CloseRPC()` 只置退出标志 + `Sleep(500)` + 关 socket，**不 join**；指令接收线程可能仍阻塞在 `Recv` | `FRRobot` 对象**在进程生命周期内不要析构**（否则 use-after-free）；**不要反复 new/delete** |
| L4 🔍 | 看门狗线程：`frame_cnt` 连续 25 次（每 8 ms 查一次 ≈ 200 ms）不变且未开重连 → 置 `g_sock_com_err` + 状态清零 | 与 R3、C5 联动 |
| L5 🔍 | `LoggerInit()` 必须在 `RPC()` **之前**调用，否则失败；8080 即时指令每条都 `logger_info` | — |
| L6 🔍 | 构造函数硬编码默认 IP `192.168.58.2` | IP 必须来自 URDF 参数 |
| L7 🔍 | 8080 即时指令通道（TCP，`/f/b` 文本帧 `/f/bIII<cnt>III<cmdID>III<len>III<content>III/b/f`）在 SDK 3.8.4 中只有 5 个方法：`StopMotion`(102 `STOP`)、`PauseMotion`(103 `PAUSE`)、`ResumeMotion`(104 `RESUME`)、`ExtAxisStopJog`(240 `StopExtAxisJog`)、`ConveyorComDetectTrigger`(1149)。`StopMotion` 的计数字段被格式串**写死为 44**（传了 `cnt` 但无 `%d`），其余正常计数；调用只写共享缓冲 `g_sendbuf` 并置 `is_sendcmd`，由发送线程每 1 ms 轮询发出，**立即返回 0、不等发送、不等回执**，连续调用会互相覆盖 | **按 SDK 3.8.4 逐字节复现**（含写死的 44，维护者决定不改）；只实现驱动用到的 `STOP`；自研实现中同步发送并等待回执（见 L9） |
| L8 📋 | **8080 同一时刻只服务一个客户端**（仿真器实测）：第二个客户端 TCP 能连上、`send` 成功，但**无任何回复**；第一个断开后，第二个**积压的指令才被执行**。即：被占用时指令**静默排队、延后执行**（延后的 PAUSE/RESUME 是危险的） | 驱动必须是该控制器上 8080 的**唯一**使用者（其他 SDK 程序、厂商工具都不能同时连）；发指令后必须等回执，超时即判 COMM 故障，并断开连接以免积压指令延后执行 |
| L9 📋 | **即时指令有回执**：仿真器对 STOP 回复 `/f/bIII44III102III1III1III/b/f`（原样带回计数与命令号，内容 `1`）；SDK 只打日志不处理。❓ 内容 `1` 的含义（成功？）及失败时的取值待确认 | 按命令号（不能靠计数，STOP 固定为 44）匹配回执 |
| L10 📋 | 8080 上只使用 SDK 3.8.4 已用的上述 5 条指令（维护者决定） | 不扩展 |
| L11 📋 | **20004 状态流同一时刻也只服务一个客户端**（仿真器实测）：驱动已占用时，第二个客户端能连上，但**收不到任何帧**（`fr_probe` 报"no state frame"）；占用者断开后恢复 | 驱动是该控制器上 20004 与 8080 的**唯一**使用者；驱动运行期间 `fr_probe`、`fr_servo_bench` 不能同时连；真机 ❓ |

## 六、固件故障行为（维护者确认 📋）

| # | 行为 | 驱动对策 |
|---|---|---|
| F1 📋 | 机器人层故障种类多：超速、超限、碰撞等（`main_code`/`sub_code`、`collisionState`） | 统一按"机器人故障"处理，错误码原样导出 |
| F2 📋 | **故障期间发送任何运动指令都会报错** | 检测到故障后**立即停发 ServoJ**；在途的那一帧报错属预期，不得再升级处理 |
| F3 📋 | **不应自动 `ResetAllError`** | 复位只能由操作者显式触发（服务调用），驱动绝不自动复位 |
| F4 📋 | **急停直接切断控制箱电源**（风扇、指示灯熄灭）：拍下期间控制器从网络上消失（ARP 无应答），状态帧里**看不到** `EmergencyStop=1`；松开后控制箱自行上电启动，报 `main_code=1`、`sub_code=149`、SI0/SI1 置位、未使能；此时 `ResetAllError` 返回 -1、状态不变（文档错误码 31：急停按钮松开，请断电重启控制箱）。断电重启后无故障并自动使能 | 驱动先看到 COMM，重连后看到 ROBOT_FAULT；恢复只靠操作者断电重启，不依赖 `ResetAllError` 在此时的行为 |
| F5 📋 | **`ResetAllError` 不会掉使能**（3.8.4 维护者确认） | 复位后无需 `RobotEnable`，但仍以状态帧 `rbtEnableState` 为准 |
| F6 📋 | `RobotEnable(0/1)` 会触发机械臂内部继电器（响声明显）。真机：下使能 → 状态帧 `rbtEnableState=0`、无故障码；`RobotEnable(1)` 后即恢复，无需 `ResetAllError` | 驱动不使能、不下使能；下使能时驱动判 ROBOT_FAULT 并停发，上使能后由操作者重新激活 |

## 七、构建

| # | 陷阱 | 驱动对策 |
|---|---|---|
| B1 🔍 | 上游 CMake：`-O0 -g`、输出到源码树 `LinuxBuild/bin`、顺带编译 `main.cpp` 测试程序、glob 收集源文件、不导出 install target | 不用上游 CMake，自写 |
| B2 🔍 | 源文件为 **GB18030 + CRLF**，中文注释在 UTF-8 下乱码 | 移植时 `iconv -f GB18030 -t UTF-8` + `dos2unix` |
| B3 🔍 | `robot_types.h` 里 `typedef int errno_t`（与 C11 Annex K / Windows 冲突），结构体 `#pragma pack(1)` | 不要在驱动公共头里暴露 SDK 头 |

## 八、末端夹爪

| # | 行为 | 对策 |
|---|---|---|
| G1 📋 | 夹爪**每个机械臂上电周期只需激活一次**：激活状态属于控制器，与 SDK 连接、客户端进程无关；控制器断电重启后才需要重新激活。**未激活时状态帧 `gripper_position` 报 0 %，与手指实际位置无关**（断电重启后实际全开、G11 失去激活后实际 100 %，都报 0 %）；SDK 的 `GetGripperCurPosition` 读的也是这个字段 | 激活前先读激活状态，已激活就不再激活；未激活时开度按不可知处理（驱动报全开：外轮廓最大，碰撞检查偏保守） |
| G2 📋 | **激活必然张开到满行程**，不论当前开度、手里是否夹着东西 | 绝不自动激活；激活只由操作者显式发起，发起前确认手里无物、张开路径无障碍 |
| G3 🔍 | 激活状态可读且不需发指令：`GetGripperActivateStatus` 只是返回状态帧的 `gripper_active`（bit0–15 对应夹爪编号 0–15）与 `gripper_fault`；激活指令为 XML-RPC `ActGripper(index, action)`（`action` 0 复位、1 激活） | 读状态帧，`fr_probe` 打印 `gripper_active`/`gripper_position`/`gripper_fault` |
| G4 📋 | **未激活时任何夹爪动作指令都只会失败**，不会隐式激活；能让夹爪张开的只有显式的 `ActGripper` | 风险集中在激活这一条指令上（见 G2） |
| G5 📋 | **真机激活**（3.8.4.2，大寰 PGI-140，夹爪编号 1）：`ActGripper(1, 1)` 约 20 ms 返回 0（不等激活完成）；约 0.9 s 内 `gripper_active` 0x0000 → **0x0001**（编号 1 对应 bit0）；夹爪实际张开到满行程，但其后 12 s 状态帧 `gripper_position` 仍为 **0%**、`gripper_motiondone` 为 0、`gripper_fault` 为 0 | 激活后、首次 `MoveGripper` 之前，状态帧里的位置**不可信**（实际全开却报 0%） |
| G6 📋 | **真机运动**：`MoveGripper(1, 50, 20, 20, 5000, 1, 0, 0, 0, 0)`（`block=1` 非阻塞）37 ms 返回 0；位置 96 → 50% 约 2.1 s（vel 20% 时约 22 %/s），到位后 `gripper_motiondone=1`。**位置 100% = 全开，0% = 闭合**；机械臂静止、无故障 | ROS 侧换算：开度 = `pos%`；完成以 `motiondone` 且位置到达为准 |
| G7 📋 | `GetGripperConfig`（XML-RPC 无参数）在本机返回 `[0, 0, 3, 0, 0]`（错误码 0、厂商 0、设备 3），后来又返回 `[0, 1, 3, 0, 0]`（厂商 1），都与 SDK 头文件的编号（大寰 = 4、PGI-140 = 0）不符，但夹爪工作正常 | 驱动不读也不写；不要按头文件编号去"纠正"配置 |
| G8 📋 | **`ServoJ` 流进行中，`MoveGripper` 不执行、也不回复**：控制器把它压到流停下（队列排空即可，不需要 `ServoMoveEnd`）之后才回复（流中途发出，7.2 s 后、流结束时才回复；停发后 26 ms 回复）并执行；客户端超时放弃的调用照样会在流停下后执行。没有流时（含只调了 `ServoMoveStart`）16–17 ms 回复 | 只在伺服流停发时发 `MoveGripper`；不要靠超时判断"没发出去"：调用失败或超时后，夹爪是否在动仍以状态帧为准，此前机械臂保持不动 |
| G9 📋 | **夹爪运动期间 `ServoJ` 照收不执行**：队列从 0 涨到 253（约 2 s，与夹爪运动时长相同），夹爪完成后才开始消费，之后队列一直满（每条命令延迟约 2 s），直到停发才排空。期间机械臂不动、无故障 | 夹爪运动期间不发 `ServoJ`；从发出 `MoveGripper` 到状态帧报完成之间，机械臂必须停着 |
| G11 📋 | **`MoveGripper` 的阻塞模式（`block = 0`）不阻塞，且使夹爪失去激活并报故障**（3.8.4.2，无伺服流，夹爪原为激活、100 %、无故障）：`MoveGripper(1, 50, 50, 20, 30000, 0, 0, 0.0, 0, 0)` 33 ms 返回 0；随后状态帧 `gripper_active` 0x0001 → **0x0000**、`gripper_fault = 1`、位置 0 %、`motion_done = 0`，至少保持数十秒；之后的 `MoveGripper` 返回 **73**（`ERR_GRIPPER_MOTION`）。夹爪实际停在原位（100 %），没有执行该命令。恢复：只需 `ActGripper(1, 1)`（6 ms 返回 0，约 20 ms 故障清除，约 0.7 s 激活位置位，帧报 0 %、未完成，同 G5），不必先复位 | **只用 `block = 1`**；完成以状态帧为准（G6、G10） |
| G10 📋 | **每次夹爪运动开始时，状态帧位置先报 0 %**：`MoveGripper` 后约 0.8 s 内 `gripper_position = 0`、`gripper_motiondone = 0`，之后约每 0.4 s 才更新一次（95 → 73 → 52 → 50），到位后 `motiondone = 1`。原因是控制器与夹爪之间的外设通信速率低（维护者）；G5（激活后报 0 %）很可能是同一现象 | 运动中、帧报 0 % 时沿用上一个已知开度（激活后则为全开）；速度按开度变化之间的时间算，不按相邻帧 |
| G12 📋 | **`MoveGripper` 的速度与力参数都生效**（维护者经 SDK 实测确认） | goal 的 `velocity`、`effort` 换算成百分比照发（`fr_robot_driver/README.md` "夹爪"）；力按被夹物设置，不要一律满量程 |
| G13 📋 | **夹持、掉落（报 3）与从夹持状态张开实测**（3.8.4.2，大寰 PGI-140，X2 实测）：① **夹住物体**：`MoveGripper(0%)` 闭合受阻停下，状态帧报 `gripper_motiondone = 2` 并持续保持；② **物体掉落（报 3）**：夹住约 6 s 后抽走物体，反向阻力消失，夹爪立即继续闭合（`motion_done` 瞬间变 0），闭合到底（0 %）未受阻触碰死区后**状态帧报 `motion_done = 3` 并持续保持**（从脱出到报 3 约 400 ms），证实大寰的掉落寄存器状态被控制器原样转发；另据观察，夹住超过约 10 s 后再抽走，夹爪保持开度与 `motion_done = 2`（❓ 未录数据）；③ **从夹持状态张开**：从 `motion_done = 2` 直接发 `MoveGripper(100%)`，序列严格为 **`2 -> 0 -> 1`**（第 1 帧由 2 变 0，运动中保持 0，全开到位后变 1） | 证实“先报 0、再报非 0”判定在从夹持状态出发时完全成立；状态机保留对 `motion_done = 3` 的处理（报 3 判定为物体掉落，action ABORTED，`gripper_object = dropped`） |
| G14 📋 | **原地运动开度序列实测**（3.8.4.2，E5）：发出的目标等于当前开度时，调用后第一帧 `motion_done = 0`，约 0.66–0.70 s 回到 1，其后位置仍报 0 % 约 200 ms 才恢复；夹住物体时原地发（7% → 7%）无此现象（位置先于完成信号恢复） | 发出的目标非 0 时，状态帧的 0 % 只在出现过非 0 位置之后才采信；否则保持运动前的开度 |
| G15 📋 | **`MoveGripper` 无法打断正在进行的夹爪运动**（3.8.4.2，大寰 PGI-140，X1 实测）：夹爪从 100 % 往 0 % 运动中途发 `MoveGripper(80%)`，**该 XML-RPC 调用在控制器端被阻塞挂起整整 1536 ms**，夹爪**绝不立即转向**，而是**走完原目标（闭合到 0 %）之后才执行新目标（张开到 80 %）**；调用在原目标走完到达 0 % 后才返回 `code = 0`；`motion_done` 全程保持为 0，直到新目标 80 % 到位才变 1 | 控制器无法中止已发出的夹爪运动。取代和取消时必须等待当前运动走完后再做后续动作；运动中发出的新 `MoveGripper` 不会被丢弃，会排在当前运动之后执行；调用方发新目标时不要在实时循环中同步调用（会阻塞 1–2 秒），须由独立工作线程异步下发 |
| G16 📋 | **`GetGripperConfig` 不是读夹爪状态的前提，读回的厂商号也与夹爪能否工作无关**（3.8.4.2，大寰 PGI-140，控制器重启后未打开夹爪页面）：不调 `GetGripperConfig`，状态帧的 `gripper_active`（`ActGripper` 后约 0.74 s 置位）、`gripper_position`、`gripper_motiondone` 都正常更新，序列同 G5、G10；调用后行为不变。`GetGripperConfig` 返回 `[0, 1, 3, 0, 0]`（company 1 = Robotiq、device 3 文档未定义），夹爪照常工作、无故障。`gripper_temp`、`gripper_voltage` 始终为 0 | 驱动不调 `GetGripperConfig`；不按读回的厂商号判断夹爪是否可用；不使用 `gripper_temp`/`gripper_voltage` |

---

## 驱动设计约束（由上述陷阱推出）

1. `read()`/`write()` **零阻塞**：不得调用任何 SDK/XML-RPC 函数，只读写共享缓冲
2. 状态：校验帧头与校验和；按接收时间判断新鲜度，超时（默认 3 个周期无新帧）→ `ERROR`
3. 命令：`on_activate` 用**已校验的真实状态**初始化命令；ServoJ 由独立发送线程以整毫秒周期按序下发（重采样补偿时钟偏差）
4. 周期：`update_rate` 整除 1000，`cmdT = 1/update_rate`，起步 125 Hz
5. 连接：客户端对象在 `on_init` 创建一次、生命周期内不替换，只在 configure / cleanup 时连接、断开；恢复走 `on_error` → 重新 configure
6. 待真机验证项（❓）在实测前**不得**作为设计前提
