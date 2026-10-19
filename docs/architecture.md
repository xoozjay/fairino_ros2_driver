# 架构

本仓库各包、进程、线程与控制器之间的关系。图用 Mermaid 写成（GitHub 与多数 Markdown 预览直接渲染）。
图只画结构与数据流，规则的理由在各自的出处：故障处理见 [fault_handling.md](fault_handling.md)，
固件行为见 [sdk_quirks.md](sdk_quirks.md)（文中 S*/G*/F* 编号），用法见各包 README。

## 1. 系统上下文

一台机械臂对应一套 ros2_control 进程与一台法奥控制器。驱动只经三个 TCP 端口与控制器通信，
不链接厂商 SDK（`libfairino`）。

```mermaid
flowchart LR
    subgraph upper["上层"]
        moveit["move_group<br/>fr_moveit_config"]
        app["应用 / 脚本<br/>action 客户端"]
        op(["操作者<br/>reset_error、gripper/activate、<br/>ros2 control CLI、急停按钮"])
    end

    subgraph host["主机：每台机械臂一个 ros2_control_node（命名空间 &lt;ns&gt;）"]
        cm["controller_manager<br/>JSB、JTC"]
        plugin["FrRobotHardwareInterface<br/>fr_robot_driver 插件"]
        hwnode["节点 &lt;tf_prefix&gt;fr_robot_hw<br/>~/status、服务、夹爪 action"]
        lib["fr_client_library<br/>纯 C++17，运行时不依赖 ROS"]
        cm <-->|"命令 / 状态接口"| plugin
        plugin --- hwnode
        plugin --> lib
    end

    subgraph box["法奥控制箱（软件版本 3.8.4）"]
        p20003["TCP 20003<br/>XML-RPC"]
        p20004["TCP 20004<br/>状态帧，8 ms"]
        p8080["TCP 8080<br/>即时指令，单客户端"]
        fw["固件运动队列<br/>最多 255 段，1 kHz 插补"]
        p20003 --> fw
        p8080 -->|STOP 清空| fw
    end

    arm["FR 机械臂"]
    grip["PGI-140 夹爪<br/>挂在控制器上"]

    moveit -->|"FollowJointTrajectory<br/>（mock：ParallelGripperCommand）"| cm
    moveit -->|ParallelGripperCommand| hwnode
    app --> cm
    app --> hwnode
    op --> hwnode
    op --> cm
    lib -->|"ServoJ、MoveGripper、<br/>配置与查询"| p20003
    p20004 -->|状态帧| lib
    lib -->|"STOP（仅无故障停用时）"| p8080
    fw --> arm
    fw -->|"MoveGripper，与 ServoJ 轮流执行（G8、G9）"| grip
    op -.->|"急停：控制箱断电（F4）"| box
```

| 端口 | 方向 | 内容 | 主机侧连接 |
|---|---|---|---|
| 20003 | 请求 / 应答 | XML-RPC over HTTP/1.1 长连接 | 三条互不阻塞：`RobotApi`（配置、服务）、`ServoSender`（ServoJ 流）、`GripperSender`（MoveGripper） |
| 20004 | 控制器推送 | 状态帧（`RobotState384`，960 字节） | `StateReceiver`；控制器只服务一个客户端 |
| 8080 | 请求 / 回执 | 即时指令，驱动只用 STOP | `InstantCommandChannel`；控制器只服务一个客户端 |

## 2. 包与依赖

```mermaid
flowchart BT
    subgraph repo["本仓库 src/fairino_ros2_driver"]
        lib["fr_client_library<br/>协议客户端库 + 工具 fr_probe、fr_servo_bench"]
        desc["fr_description<br/>参数化 URDF 宏，10 个型号"]
        grip["gripper_dahuan_pgi140<br/>夹爪 URDF 宏"]
        drv["fr_robot_driver<br/>硬件插件、URDF ros2_control 宏、launch、控制器配置"]
        mv["fr_moveit_config<br/>SRDF、OMPL + Pilz、move_group launch"]
    end

    subgraph ext["外部（ROS 2 Jazzy）"]
        rc["ros2_control 4.48.1<br/>hardware_interface、controller_manager"]
        ctl["joint_state_broadcaster<br/>joint_trajectory_controller<br/>parallel_gripper_controller（mock）"]
        rcl["rclcpp、rclcpp_action<br/>control_msgs、diagnostic_msgs、std_srvs"]
        mi["MoveIt 2<br/>pilz_industrial_motion_planner"]
    end

    drv -->|链接| lib
    drv -->|xacro 引用| desc
    drv -.->|运行时| grip
    drv --> rc
    drv --> rcl
    drv -.->|运行时| ctl
    mv --> desc
    mv --> drv
    mv --> grip
    mv --> mi
```

- `fr_client_library` 只依赖 C++17 与 POSIX；ament_cmake 只负责构建、安装与测试。
- `fr_robot_driver/urdf/fr_controlled.urdf.xacro` 把 `fr_description` 的机械臂宏、可选的夹爪宏与
  `fr_ros2_control.xacro` 拼成 controller_manager 读取的一份 URDF；mock 与真机只差 `<hardware>` 一节。

## 3. ROS 图（单台机械臂，真机分支）

`<ns>` 为 launch 的 `namespace`，`<p>` 为 `tf_prefix`。多机时每台机械臂各有一份下图，靠命名空间与
`tf_prefix` 隔离（第 10 节）。

```mermaid
flowchart LR
    subgraph proc["ros2_control_node（/&lt;ns&gt;/controller_manager）"]
        direction TB
        subgraph rt["实时控制循环，1000 / cmd_t_ms Hz"]
            jsb["joint_state_broadcaster"]
            jtc["joint_trajectory_controller<br/>speed_scaling 接口"]
            hw[["FrRobotHardwareInterface<br/>组件 &lt;p&gt;fr_robot"]]
        end
        node["节点 /&lt;ns&gt;/&lt;p&gt;fr_robot_hw<br/>挂在 controller_manager 的执行器上"]
        hw --- node
    end

    rsp["robot_state_publisher"]
    mg["move_group"]

    jtc -->|"&lt;p&gt;*_joint/position（命令）"| hw
    hw -->|"关节 position、velocity<br/>&lt;p&gt;fr_status/fault_class<br/>&lt;p&gt;speed_scaling/speed_scaling_factor<br/>&lt;p&gt;left_finger_joint/position"| jsb
    hw -->|状态接口| jtc

    jsb -->|"/&lt;ns&gt;/joint_states<br/>/&lt;ns&gt;/dynamic_joint_states"| rsp
    jsb -->|"/&lt;ns&gt;/joint_states"| mg
    rsp -->|"/tf、/tf_static"| mg
    mg -->|"action /&lt;ns&gt;/joint_trajectory_controller/<br/>follow_joint_trajectory"| jtc
    mg -->|"action /&lt;ns&gt;/gripper_controller/gripper_cmd"| node

    node -->|"~/status<br/>DiagnosticStatus，transient_local"| obs(["rqt_runtime_monitor<br/>上层监控"])
    opr(["操作者"]) -->|"~/reset_error<br/>~/gripper/activate（Trigger）"| node
```

| 接口 | 真机 | mock（`mock_components/GenericSystem`） |
|---|---|---|
| 6 个关节：命令 `position`，状态 `position`、`velocity` | 插件 | 回环 |
| `<p>fr_status/fault_class` | 0 正常 / 1 机器人故障 / 2 急停 / 3 通信 | 恒 0（`mock_gpio_commands:=true` 时可写） |
| `<p>speed_scaling/speed_scaling_factor` | 1.0；夹爪忙时 0.0 | 恒 1.0（同上） |
| `<p>left_finger_joint`（`gripper:=pgi140`） | 只有状态接口 | 另有 `position`、`max_effort` 命令接口 |
| 夹爪 action `gripper_controller/gripper_cmd` | 插件节点提供 | `parallel_gripper_action_controller` 提供 |
| `~/status`、`~/reset_error`、`~/gripper/activate` | 插件节点提供 | 无 |

## 4. 插件内部结构

插件 `FrRobotHardwareInterface` 负责生命周期、线程与 I/O；规则判断放在不做 I/O、不读时钟的纯逻辑类里，
可脱离 ROS 和网络单独测试。

```mermaid
flowchart LR
    subgraph plugin["FrRobotHardwareInterface"]
        direction TB
        lc["生命周期回调<br/>on_init … on_cleanup"]
        svc["服务与 action 回调<br/>reset_error、gripper/activate、<br/>gripper_cmd、20 ms 目标定时器"]
        rw["read() / write()<br/>实时线程"]
        diag["publishDiagnostics()<br/>100 ms 定时器"]
        cln["清理线程<br/>startCleanup / monitorLink"]
    end

    subgraph pure["纯逻辑：无 I/O、不读时钟"]
        direction TB
        pre["preconditions.hpp<br/>checkVersion、checkModel、checkFlange、<br/>checkActivatable、refuseGripperActivation、decideReset"]
        kin["Kinematics<br/>FK，核对 flange_cur_pos"]
        goals["GripperGoals<br/>接受、抢占、取消、作废、结果映射"]
        gate["ArmGate<br/>抽取周期、步长检查、夹爪忙时保持"]
        motion["GripperMotion<br/>夹爪完成判定与开度推断"]
        status["status_report.hpp<br/>effectiveFault、StatusTracker"]
        pre --> kin
    end

    subgraph libc["fr_client_library 对象：on_init 创建，生命周期内不替换"]
        direction TB
        api["RobotApi api_<br/>api_mutex_ 保护"]
        inst["InstantCommandChannel"]
        recv["StateReceiver"]
        send["ServoSender"]
        gs["GripperSender（有夹爪时）"]
    end

    lc --> pre
    svc --> pre
    svc --> goals
    rw --> goals
    rw --> gate
    rw --> motion
    rw --> status
    diag --> status

    lc ==>|"连接、启动、停止"| libc
    svc -->|"ResetAllError、ActGripper"| api
    rw -->|"tryLatest()"| recv
    rw -->|"enqueue()、halt()"| send
    rw -->|"post()、discard()"| gs
    diag -->|"latest()"| recv
    cln -->|STOP| inst
    cln -->|"stop()：ServoMoveEnd"| send
    cln -->|"每 2 s 重连 20004"| recv
```

### fr_client_library 分层

```mermaid
flowchart BT
    sock["TcpSocket<br/>非阻塞，所有操作带超时"]
    http["HttpConnection<br/>HTTP/1.1 长连接，按 Connection: close 与空闲轮换"]
    codec["xmlrpc_codec<br/>XmlRpcValue 编解码，格式同 SDK"]
    parser["FrameParser<br/>搜 0x5A5A、按 data_len 收帧、校验和、重同步"]
    state["RobotState384<br/>960 字节，static_assert 锁定偏移"]
    ver["software_version<br/>白名单 3.8.3 / 3.8.4，版本 → 帧长"]
    subgraph know["协议知识：插件的纯逻辑与 fr_probe 使用"]
        fault["fault_state<br/>FaultClass、classifyState"]
        model["robot_model<br/>型号字符串 → fr_description 机型"]
    end
    spsc["SpscQueue<br/>有界无锁，单生产者单消费者"]
    loop["DepthLoop<br/>队列深度 PI 环"]

    api["RobotApi<br/>类型化 XML-RPC 调用"]
    recv["StateReceiver<br/>接收线程，保留最新一帧"]
    send["ServoSender<br/>发送线程：预填、重采样、空闲停发"]
    gs["GripperSender<br/>发送线程：只保留最新一条"]
    inst["InstantCommandChannel<br/>STOP，同步等回执"]

    http --> sock
    api --> http
    api --> codec
    recv --> sock
    recv --> parser
    parser --> state
    parser --> ver
    fault --> state
    send --> api
    send --> spsc
    send --> loop
    gs --> api
    inst --> sock
    api --> ver
```

## 5. 线程模型

```mermaid
flowchart LR
    subgraph T1["实时控制线程（controller_manager）"]
        read["read()"]
        upd["控制器 update()"]
        write["write()"]
        cb["on_deactivate / on_error<br/>在 read()/write() 内同步调用"]
        read --> upd --> write
    end

    subgraph T2["执行器线程（MultiThreadedExecutor）"]
        lcy["on_configure / on_activate / on_cleanup<br/>（经 controller_manager 服务）"]
        srv["~/reset_error"]
        act["~/gripper/activate<br/>独立回调组，最多等 3 s"]
        goal["gripper_cmd 回调<br/>20 ms 目标定时器"]
        stat["100 ms 状态定时器 → ~/status"]
    end

    subgraph T3["StateReceiver 线程"]
        rx["TCP 20004 → FrameParser"]
    end
    subgraph T4["ServoSender 线程（可选 SCHED_FIFO）"]
        tx["重采样 → ServoJ（20003）"]
    end
    subgraph T5["GripperSender 线程"]
        gx["MoveGripper（20003）"]
    end
    subgraph T6["清理线程（短命）"]
        cx["STOP（8080）→ ServoMoveEnd<br/>出错后：关闭连接，每 2 s 重连 20004"]
    end

    newest[("最新帧<br/>newest_mutex_")]
    q[("SpscQueue<br/>64 条，无锁")]
    slot[("待发夹爪命令<br/>一个槽，mutex + cv")]
    atom[("原子量<br/>fault_class_、gripper_busy_、<br/>gripper_epoch_、gripper_finished_")]
    gm[("gripper_goal_mutex_<br/>实时线程只 try_lock")]

    rx -->|写| newest
    newest -->|"tryLatest()，不等待"| read
    newest -->|"tryLatest()：队列深度、是否到位"| tx
    newest -->|"latest()"| stat
    newest -->|"latest()"| act
    write -->|"enqueue()"| q --> tx
    write -->|"post()"| slot --> gx
    read <--> atom
    write <--> atom
    goal <--> atom
    write -.->|try_lock| gm
    goal --> gm
    cb -->|启动| T6
```

- `read()`/`write()` 从不阻塞在网络或锁上：状态用 `tryLatest()`（接收线程正在写入的那一刻沿用上一帧），
  伺服命令进无锁队列，夹爪命令进一个非阻塞的槽，目标表只 `try_lock`。
- `on_deactivate`/`on_error` 跑在实时线程上，只置标志、`halt()`；STOP、`ServoMoveEnd`、关闭连接交给清理线程，
  下一次 configure/activate 先等它结束。
- 三条 20003 连接各属一个线程，ServoJ 流不会被服务或夹爪调用阻塞。

## 6. 一个控制周期与伺服链路

```mermaid
sequenceDiagram
    autonumber
    participant JTC as joint_trajectory_controller
    participant HW as 插件 read()/write()
    participant Gate as ArmGate
    participant Q as SpscQueue
    participant SS as ServoSender 线程
    participant FW as 控制器固件
    participant SR as StateReceiver 线程

    Note over HW,SR: 实时线程每周期：read → update → write
    SR->>HW: 最新状态帧（tryLatest）
    HW->>HW: 判 COMM（帧过期 / 断线 / 发送线程死）<br/>effectiveFault → fault_class
    HW->>JTC: 关节位置与速度、speed_scaling
    JTC->>HW: 位置命令（rad）
    HW->>Gate: sendCycle()? decide(命令 deg, 夹爪忙)
    alt 步长超限
        Gate-->>HW: Refuse
        HW->>SS: halt()
        Note over HW: ProtectiveStop::StepLimit<br/>返回 DEACTIVATE → INACTIVE
    else 放行
        Gate-->>HW: Send(target)
        HW->>Q: enqueue(target)
    end
    loop 每 cmdT·(1+eps)
        Q->>SS: 取命令（攒够 kLead=3 条才开始）
        SS->>SS: 折线上线性插值重采样<br/>DepthLoop 按 mc_queue_len 调 eps（±1%）
        SS->>FW: ServoJ(q, cmdT)（20003）
    end
    FW->>FW: 队列目标深度 = prefill（3）<br/>1 kHz 插补执行
    FW-->>SR: 状态帧（20004，8 ms）<br/>jt_cur_pos、mc_queue_len、故障位
```

- 命令到实际运动的跟随滞后约 116–133 ms（S19），其中约 48 ms 是 `kLead` 3 条与 `prefill` 3 条的缓冲。
- 控制周期快于 cmdT 时按整数倍抽取（`ratio = update_rate × cmd_t_ms / 1000`）。
- 停发 ServoJ 不会让机械臂立即停下，固件会走完队列里的段；无故障的主动停用先发 STOP（8080）清空队列。

### ServoSender 的状态

```mermaid
stateDiagram-v2
    [*] --> Stopped
    Stopped --> WaitLead: start(actual)<br/>连接、ServoMoveStart
    WaitLead --> Prefill: 主机队列攒够 kLead 条
    Prefill --> Streaming: 发原地不动段<br/>直到估计深度达到 prefill
    Streaming --> Streaming: 每 cmdT·(1+eps) 发一段
    Streaming --> Starved: 生产者迟到，窗口不足
    Starved --> Streaming: 命令到达，不补发欠下的段
    Streaming --> Paused: 要发的点与上一段相同<br/>且状态帧显示已到位（S18）
    Paused --> WaitLead: 收到不同的命令
    Streaming --> Halted: halt()（故障）
    Paused --> Halted: halt()
    Prefill --> Halted: halt()
    Halted --> Halted: 丢弃主机队列
    Halted --> Stopped: stop()：ServoMoveEnd
    Streaming --> Dead: 传输错误（不重试）
    Dead --> Stopped: stop()
    Stopped --> [*]
```

`Paused` 时控制器队列排空，`MoveGripper` 才会被执行（G8）；从 `Paused` 恢复与 `start()` 相同，先攒 `kLead` 条再预填。
`Starved` 只是等数据，不重新预填。`Dead` 由下一次 `read()` 以 `CommReason::ServoFailed` 报为通信故障。

## 7. read() / write() 决策

```mermaid
flowchart TD
    R0([read]) --> R1{"tryLatest 拿到新帧?"}
    R1 -->|否，沿用上一帧| R2
    R1 -->|是| R2{"帧过期 / 20004 断开 /<br/>ServoSender 已死?"}
    R2 -->|是| RC["fault_class = 3 COMM<br/>return ERROR → on_error → UNCONFIGURED"]
    R2 -->|否| R3["effectiveFault(帧, 保护停止)<br/>发布关节、fault_class"]
    R3 --> R4["有夹爪：GripperMotion.frame()<br/>开度、完成判定 → gripper_finished_"]
    R4 --> R5["speed_scaling = 夹爪忙 ? 0 : 1"]
    R5 --> R6{"ACTIVE 且有故障?"}
    R6 -->|是| R7["sender.halt()"] --> ROK([return OK])
    R6 -->|否| ROK

    W0([write]) --> W1{"ACTIVE?"}
    W1 -->|否| WOK([return OK])
    W1 -->|是| W2{"fault_class ≠ 0?"}
    W2 -->|是| WD["halt()、丢弃夹爪命令<br/>return DEACTIVATE → INACTIVE"]
    W2 -->|否| W3["读 6 个位置命令（rad → deg）<br/>writeGripper：条件满足则 post(MoveGripper)"]
    W3 --> W4{"本周期是伺服段?<br/>（ratio 抽取）"}
    W4 -->|否| WOK
    W4 -->|是| W5{"ArmGate.decide"}
    W5 -->|"Refuse：步长超限或非有限值"| WS["ProtectiveStop::StepLimit<br/>fault_class = 1，halt()<br/>return DEACTIVATE"]
    W5 -->|"Send（夹爪忙时为上一段）"| W6{"enqueue 成功?"}
    W6 -->|否| WC["fault_class = 3 COMM<br/>return ERROR"]
    W6 -->|是| W7["gate.sent(target)"] --> WOK
```

故障分级、生命周期状态机与恢复流程见 [fault_handling.md](fault_handling.md) 第二至五节。

### 生命周期回调做什么

| 回调 | 线程 | 连接与线程 | 检查（`preconditions.hpp`） |
|---|---|---|---|
| `on_init` | 主线程 | 创建全部客户端对象（不连接）与节点 `<p>fr_robot_hw`（服务、action、定时器） | URDF 接口与限位、参数范围、`update_rate × cmd_t_ms` 整除 |
| `on_configure` | 执行器 / 启动 | 连 20003 → 设并回读状态周期 → 启动 `StateReceiver` 等首帧 → 读负载与工具（只读）→ 连 8080 → 启动 `GripperSender` | 版本白名单、型号一致、法兰 FK 核对 |
| `on_activate` | 执行器 / launch 显式激活 | 命令置为实际位置，`ServoSender::start(actual)`，新建 `ArmGate` | 帧新鲜、无故障、已使能、法兰 FK 再核对 |
| `on_deactivate` | 实时线程 | `halt()`；清理线程：无故障时 STOP，再 `ServoMoveEnd` | — |
| `on_error` | 实时线程 | `halt()`；清理线程：`ServoMoveEnd`，关闭全部连接，之后每 2 s 只读重连 20004 | — |
| `on_cleanup` / `on_shutdown` | 执行器 | 结束清理线程，关闭全部连接 | — |

## 8. 夹爪 action（真机）

```mermaid
sequenceDiagram
    autonumber
    participant C as action 客户端
    participant EX as 执行器线程<br/>action 回调、20 ms 定时器
    participant G as GripperGoals
    participant RT as 实时线程 read()/write()
    participant GS as GripperSender 线程
    participant FW as 控制器

    C->>EX: goal（position m、effort N）
    EX->>EX: 硬件 ACTIVE 且夹爪可用? 否则 REJECT
    EX->>G: accept → 旧目标 ABORTED（抢占）
    loop 每个控制周期
        RT->>G: maySend(epoch、可用、不忙、<br/>伺服流已停发且固件队列空、机械臂不动)
    end
    RT->>GS: post(MoveGripper %)（不阻塞）
    RT->>RT: gripper_busy = Moving<br/>speed_scaling = 0，JTC 轨迹时间停住
    GS->>FW: MoveGripper（独立 20003 连接，block = 1）
    FW-->>RT: 状态帧 gripper_motiondone、gripper_position
    RT->>RT: GripperMotion：标志位与开度 300 ms 不变 → 结果<br/>写 gripper_finished_，gripper_busy = Idle
    EX->>G: outcome(gripper_finished_)
    alt 1 到位 / 2 夹住
        EX-->>C: SUCCEEDED（stalled = 2）
    else 3 失去物体 / 超时 / 作废
        EX-->>C: ABORTED
    end
```

`gripper_epoch_` 在硬件激活或停用、出错、夹爪变为不可用、夹爪激活时递增，在此之前接受的目标一律作废、不再执行。
夹爪运动期间机械臂命令保持上一段（G9），夹爪完成后才放行。

## 9. 启动顺序（真机分支）

```mermaid
sequenceDiagram
    autonumber
    participant L as fr_control.launch.py
    participant CM as ros2_control_node
    participant P as 插件
    participant SP as spawner

    L->>L: xacro → URDF；替换 controllers.yaml 的 __PREFIX__ / __NS__
    L->>CM: 启动（update_rate = 1000 / cmd_t_ms，<br/>enforce_command_limits，组件初始状态 inactive）
    CM->>P: on_init → on_configure
    L->>SP: joint_state_broadcaster
    L->>CM: ros2 control set_hardware_component_state #lt;p#gt;fr_robot active
    CM->>P: on_activate（前置条件不满足只报错，进程与 reset_error 仍在线）
    L->>SP: 激活命令退出后：joint_trajectory_controller
```

mock 分支没有显式激活这一步，同时拉起 JTC（有夹爪时加 `gripper_controller`）。让硬件先停在 `inactive` 的原因见
`fr_robot_driver/README.md`"启动顺序"。

## 10. 多机单元

```mermaid
flowchart TB
    subgraph cell["dual_fr_cell.launch.py"]
        subgraph A["命名空间 /alice，tf_prefix alice_"]
            a_cm["controller_manager + 插件"]
            a_rsp["robot_state_publisher"]
        end
        subgraph B["命名空间 /bob，tf_prefix bob_"]
            b_cm["controller_manager + 插件"]
            b_rsp["robot_state_publisher"]
        end
        t_rsp["/table/robot_state_publisher<br/>台面"]
        rviz["RViz（dual_cell.rviz）"]
    end
    tf[("/tf：world → alice_base_link、bob_base_link")]
    ca["控制器 alice_ip"]
    cb["控制器 bob_ip"]

    a_cm <--> ca
    b_cm <--> cb
    a_rsp --> tf
    b_rsp --> tf
    t_rsp --> tf
    tf --> rviz
```

每台机械臂由 `fr_control.launch.py` include 一次：话题、服务、action 靠命名空间隔离，TF 帧与关节名靠 `tf_prefix`
隔离，各自一个 controller_manager 与插件实例，连各自的控制器。ros2_control 不做碰撞检查，双臂之间的安全靠规划层。
