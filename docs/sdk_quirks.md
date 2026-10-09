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
| C7 📋 | **服务端支持 HTTP keep-alive**：仿真器响应头 `Server: Xmlrpc-c_Abyss/1.54.5`、`Connection: Keep-Alive`、`Keep-Alive: timeout=15, max=30`——**每条连接最多 30 个请求，空闲 15 s 由服务端关闭**；SDK 不用纯属 SDK 的问题。❓ 真机控制器配置待确认；仿真器在本机运行，延迟数据无参考价值 | 自研客户端用长连接；满 30 个请求或空闲接近 15 s 时，在两次请求之间换一条新连接（不重发可能已送达的请求）。伺服流约每 240 ms 有一次调用含 TCP 握手（亚毫秒级），由固件队列吸收 |
| C9 📋 | **keep-alive 实测（仿真器）**：`Keep-Alive: max=30` 是**静态值，不递减**；第 30 个请求的响应头变为 `Connection: close`（且无 `Keep-Alive` 头），随后服务端关闭连接；空闲 15 s 后服务端关闭，在陈旧连接上发送会成功但读到 EOF（无法判断请求是否已被处理）。响应头名为 `Content-length`（小写 l） | 只在两次请求**之间**轮换：遵守 `Connection: close`，空闲接近超时（留 2 s 余量）时先换新连接；请求中途出错 → 关闭并上报，**不重试**（可能已送达）。已在仿真器上连续 70 次请求验证 |
| C8 ❓ | 服务端处理单次 XML-RPC 调用的开销未知——长连接只省掉握手，不省服务端处理 | 伺服周期须在真机实测后确定 |

## 三、伺服（ServoJ）

| # | 陷阱 | 驱动对策 |
|---|---|---|
| S1 📋 | **固件以 1 kHz 运行，`cmdT` 向上取整到整毫秒**（1.5 ms → 2 ms） | `cmdT` 只用整毫秒；`update_rate` 必须整除 1000（1000/500/250/200/125/100…） |
| S2 🔍 | 头文件注释"建议范围 [0.001~0.0016]"疑为笔误；SDK 自带示例全部用 `cmdT = 0.008`（125 Hz） | 起步 125 Hz，实测后再调 |
| S3 ❓ | `cmdT` 与实际下发周期不一致时的行为（插补/抖动/超速）未知 | 保证 `cmdT == 1/update_rate`，实测确认 |
| S4 ❓ | 伺服流中断（不再发 ServoJ）时机器人如何处理（保持/超时停止/报错）未知 | 实测；驱动侧必须有发送超时看门狗 |
| S5 🔍 | 3.8.4 的 `Servo*` **没有 `comType` 参数**，只能走 XML-RPC；UDP 20007 伺服通道 3.9.4 才有 | 不可用，别抄新版代码 |
| S6 🔍📋 | SDK 示例把 `ServoJ` 包在 `ServoMoveStart()`/`ServoMoveEnd()` 之间，但 **`ServoMoveStart`/`ServoMoveEnd` 在 3.8.x 固件层面都是空语义指令**（维护者确认）——**不要依赖这一实现细节**，仍按 API 约定成对调用；`axisPos` **会被解引用，不可传 nullptr**（传全零） | 不要把"已调用 ServoMoveEnd"当作机器人已停止的依据 |
| S7 🔍 | 单位是**度**（关节）、mm（外部轴）；ROS 是弧度 | 在插件边界统一换算 |
| S8 🔍 | 每次 `ServoJ`/`ServoMoveStart` 先查 `GetSafetyCode()`：SI0/SI1 安全停止时返回 **99**——判断依据是可能撕裂/过期的状态包（见 R2） | 99 → 状态接口报安全停止 + `ERROR` |
| S9 🔍 | 3.8.4 已有力矩伺服 `ServoJTStart/ServoJT/ServoJTEnd`（`interval` 1–8 ms，Nm） | 将来 effort 命令模式可用 |

## 四、状态通道（TCP 20004）

| # | 陷阱 | 驱动对策 |
|---|---|---|
| R1 🔍 | 控制器**推送全量** `ROBOT_STATE_PKG`（960 B），周期由 `SetRobotRealtimeStateSamplePeriod(ms)` 设定（示例用 10）；仿真器默认 **8 ms**（`GetRobotRealtimeStateSamplePeriod` 读回与帧间隔一致）；❓ 最小值与真机默认值待测 | 启动时显式设置并回读 `GetRobotRealtimeStateSamplePeriod` |
| R2 🔍 | **无锁**：接收线程 `memcpy` 写入，`GetRobotRealTimeState` 同时 `memcpy` 读出 → **撕裂读** | 自研接收器用 seqlock / `realtime_tools::RealtimeBuffer`；或至少读两次比较 `frame_cnt` |
| R3 🔍 | **出错时 SDK 把状态包 `memset` 成 0**（`CloseRPC`、指令线程出错、看门狗）；`RecvPkg` 帧头非 `0x5A5A` 时**返回 0 且缓冲为全零**，状态线程照样拷贝 → **关节位置读成全 0 却像有效数据** | 每次读状态都校验 `frame_head == 0x5A5A` 和 `frame_cnt` 在变化；**绝不用全零状态初始化命令**（否则激活瞬间机械臂冲向零位） |
| R4 🔍 | 帧同步不会找 `0x5A5A` 重新对齐：一旦错位就持续出错 | 自研接收器按帧头搜索重同步 |
| R5 🔍📋 | `frame_cnt` 是 `uint8_t`（256 回绕）；仿真器上**每帧递增 2**（5, 7, 9…） | 判断"新帧"用 `!=`，不要假设 +1，也不要比大小 |
| R6 🔍 | 单位：`jt_cur_pos` 度，`jt_cur_tor` Nm；❓ `actual_qd` 推测为 deg/s 待确认 | 同 S7 |

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

## 六、固件故障行为（维护者确认 📋）

| # | 行为 | 驱动对策 |
|---|---|---|
| F1 📋 | 机器人层故障种类多：超速、超限、碰撞等（`main_code`/`sub_code`、`collisionState`） | 统一按"机器人故障"处理，错误码原样导出 |
| F2 📋 | **故障期间发送任何运动指令都会报错** | 检测到故障后**立即停发 ServoJ**；在途的那一帧报错属预期，不得再升级处理 |
| F3 📋 | **不应自动 `ResetAllError`** | 复位只能由操作者显式触发（服务调用），驱动绝不自动复位 |
| F4 📋 | **急停拍下后只能硬件重启**（控制器断电重启） | 急停 = 终态：拒绝一切复位请求；重启后控制器断线，走通信故障 → 重连流程 |
| F5 📋 | **`ResetAllError` 不会掉使能**（3.8.4 维护者确认） | 复位后无需 `RobotEnable`，但仍以状态帧 `rbtEnableState` 为准 |

## 七、构建

| # | 陷阱 | 驱动对策 |
|---|---|---|
| B1 🔍 | 上游 CMake：`-O0 -g`、输出到源码树 `LinuxBuild/bin`、顺带编译 `main.cpp` 测试程序、glob 收集源文件、不导出 install target | 不用上游 CMake，自写 |
| B2 🔍 | 源文件为 **GB18030 + CRLF**，中文注释在 UTF-8 下乱码 | 移植时 `iconv -f GB18030 -t UTF-8` + `dos2unix` |
| B3 🔍 | `robot_types.h` 里 `typedef int errno_t`（与 C11 Annex K / Windows 冲突），结构体 `#pragma pack(1)` | 不要在驱动公共头里暴露 SDK 头 |

---

## 驱动设计约束（由上述陷阱推出）

1. `read()`/`write()` **零阻塞**：不得调用任何 SDK/XML-RPC 函数，只读写共享缓冲
2. 状态：校验帧头 + `frame_cnt` 新鲜度；超时（如 > 3 个周期无新帧）→ `ERROR`
3. 命令：`on_activate` 用**已校验的真实状态**初始化命令；ServoJ 由独立发送线程以整毫秒周期下发，"最新命令覆盖"
4. 周期：`update_rate` 整除 1000，`cmdT = 1/update_rate`，起步 125 Hz
5. 连接：一个进程一个 `FRRobot`/客户端对象，生命周期内不析构；恢复走 `on_error` → 重连
6. 待真机验证项（❓）在实测前**不得**作为设计前提
