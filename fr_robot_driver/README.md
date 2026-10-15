# fr_robot_driver

ros2_control bringup for Fairino 协作机器人，对标 `ur_robot_driver`（见
`references/Universal_Robots_ROS2_Tutorials` 的 `my_robot_cell_control` 模式）。

## 两条管线

1. **mock**：`use_mock_hardware:=true`（默认），`mock_components/GenericSystem`。
   `joint_state_broadcaster` + `joint_trajectory_controller` 完整可用。
2. **真机 / 仿真器**：`use_mock_hardware:=false`，`fr_robot_driver/FrRobotHardwareInterface`
   （C++ 插件，依赖 `fr_client_library`，软件版本 3.8.4；3.8.3 仿真器已验证）。
   两条管线暴露**同样的** `fr_status/fault_class` 状态接口，上层不必区分。

### 真机 / 仿真器

```bash
# 仿真器（SimMachine 3.8.3）或真机；控制器管理器按 1000/cmd_t_ms Hz 运行（默认 125 Hz）
ros2 launch fr_robot_driver fr_control.launch.py model:=fairino5_v6 use_mock_hardware:=false \
  ip:=192.168.58.2

# 双机：两台控制器地址必须显式给出
ros2 launch fr_robot_driver dual_fr_cell.launch.py use_mock_hardware:=false \
  alice_ip:=<ip> bob_ip:=<ip>
```

**机型必须和控制器一致**：launch 的 `model` 默认是 `fairino20_v6`，而仿真器是 FR5，所以要写 `model:=fairino5_v6`。
插件在 configure 时核对控制器报告的型号，不一致就拒绝；再用 `flange_cur_pos` 与自家 FK 交叉检查（位置 ≤ 10 mm、
姿态 ≤ 1°）。相关参数：`allow_unknown_model`（控制器型号不在表里时放行，法兰检查仍有效）、`fk_check`、
`fk_position_tolerance_mm`、`fk_angle_tolerance_deg`。

launch 参数：`ip`、`cmd_t_ms`（ServoJ 的 cmdT，整毫秒，默认 8）、`prefill`（启动时的原地不动段数，也是固件队列的目标深度，默认 3）、
`allow_unverified_version`、`activate_hardware`（默认 true：启动后自动激活硬件）。

**启动顺序**：真机分支让硬件先停在 `inactive`，launch 再显式激活，成功后才启动轨迹控制器。
这样启动时若有急停/故障/未使能，激活失败只是一条错误日志，控制器管理器和 `reset_error` 服务仍然在线
（若交给控制器管理器自己激活，ros2_control 会在失败时直接 `abort`）。

### 故障与恢复（全部由操作者发起，驱动绝不自动复位）

| 现象 | 驱动做什么 | 恢复 |
|---|---|---|
| 固件故障 / 碰撞 / 安全停止 / 掉使能 | 立即停发，硬件 → `inactive`，`fr_status/fault_class = 1`，故障码原样报在 `~/status` | 排除原因 → `reset_error` → 激活硬件 → 激活控制器 |
| 命令超过单周期步长（URDF 速度限位 × cmdT） | 拒绝该命令（固件从未见到），同上，`main_code` 仍为 0 | 同上（`reset_error` 无事可做，可省） |
| 急停（本硬件：控制箱断电，见下） | 先是通信中断；控制器自行上线后状态帧报 `main_code=1 sub_code=149`，即 `fault_class = 1`。`reset_error` 拒绝（硬件已 `unconfigured`） | 控制箱断电重启（sdk_quirks F4），之后重新 configure → activate |
| 状态帧真的报 `EmergencyStop=1` | 同固件故障，`fault_class = 2`；`reset_error` **拒绝** | 只能硬件重启控制器，之后重新 configure |
| 通信中断（状态帧超时/连接断开/ServoJ 连接失败） | 硬件 → `unconfigured`，`fault_class = 3`；`reset_error` 拒绝；之后只读重连状态流（见下） | 确认控制器在线后重新 configure → activate |

本硬件上急停会切断控制箱电源（sdk_quirks F4），状态帧里从未出现 `EmergencyStop=1`，所以急停表现为
`fault_class` 3 → 1；2 只在状态帧真的报急停时出现。

**状态话题**：插件节点 `<ns>/<tf_prefix>fr_robot_hw` 发布 `~/status`（`diagnostic_msgs/msg/DiagnosticStatus`，
QoS `transient_local` 深度 1，rqt_runtime_monitor 可看）。它不依赖 `joint_state_broadcaster`：通信中断时 ros2_control
会把硬件转 `unconfigured` 并停用 broadcaster，`fr_status` 随之停发，而 `~/status` 在硬件任何生命周期状态下都在发
（状态变化时立即发，否则 1 Hz）。

| 项 | 内容 |
|---|---|
| `level` | OK；WARN：未连接，或控制器可达但硬件 `unconfigured`；ERROR：任一故障 |
| `message` | 一句话，如 `robot fault: main_code=1 sub_code=149, safety stop SI0, ...`、`communication fault: no state frame in time (link to the controller interrupted; possible causes: ...)`。链路中断只说"链路中断"，可能原因含控制器掉电（如急停），不断言是网络故障 |
| `values` | `fault_class`、`main_code`、`sub_code`、`enabled`、`estop`、`safety_stop0/1`、`servo_queue_len`（固件 ServoJ 队列占用；时钟偏差补偿使它稳态在 `prefill` 到 `prefill + 1` 之间）（无状态帧时为 `unknown`）；`hardware_state`（unconfigured/inactive/active/finalized）；`link`（up/down）；`last_frame_age_ms`；`since`（当前状态开始时刻，UTC）；`previous`（上一个故障及其起止时刻） |

**通信中断之后**：后台线程每 2 s 重连 20004 状态流（只读：不连 20003、8080，不发 ServoJ、不复位），
控制器重新上线后 `~/status` 即反映它自报的状态（如 1/149）。硬件仍是 `unconfigured`，恢复运动仍须操作者
configure → activate；`configure`/`cleanup`/退出时该线程结束并释放 20004。

```bash
ros2 service call /<ns>/<tf_prefix>fr_robot_hw/reset_error std_srvs/srv/Trigger   # 例：/fr_robot_hw/reset_error
ros2 control set_hardware_component_state <tf_prefix>fr_robot active
ros2 control set_controller_state joint_trajectory_controller active
```

ros2_control 状态接口只有 `<tf_prefix>fr_status/fault_class` 与 `<tf_prefix>speed_scaling/speed_scaling_factor`（经
`joint_state_broadcaster` 发布在 `dynamic_joint_states`），其余都在 `~/status`。`speed_scaling_factor` 平时为 1.0，
真机上夹爪忙时为 0.0：`joint_trajectory_controller` 的轨迹时间随之停住，轨迹暂停、夹爪完成后从原处继续（对标 UR 驱动）。

可选 `rt_priority`（1–99，默认 0 = 关）：伺服发送线程用 `SCHED_FIFO`；没有权限（`CAP_SYS_NICE`、rtprio 限额）时告警并照常运行。
不是必需的：固件队列能吸收约 `prefill × cmd_t_ms` 的调度抖动。

### 注意

- **同一控制器上 20004 和 8080 都只服务一个客户端**：驱动运行时，`fr_probe`、`fr_servo_bench` 等工具收不到状态帧。
- 控制器管理器的 `update_rate` 必须满足 `update_rate × cmd_t_ms / 1000` 为 ≥ 1 的整数，否则插件拒绝初始化；
  更快的控制周期按整数倍抽取（500 Hz、8 ms → 每 4 个周期发 1 条）。
- `ServoJ` 不受全局速度/加速度限制（sdk_quirks S14）：保护靠 URDF 限位、`enforce_command_limits`（真机分支默认开）
  和插件的步长检查。
- mock 分支加 `mock_gpio_commands:=true` 会给 `fr_status/fault_class` 与 `speed_scaling/speed_scaling_factor` 加回环
  命令接口，用来演练上层的故障处理与轨迹暂停。

### 夹爪（大寰 PGI-140）

`gripper:=pgi140` 把 `gripper_dahuan_pgi140` 挂到 `tool0`，ros2_control 多一个关节 `<tf_prefix>left_finger_joint`
（开度 = 单侧手指行程，0 闭合 – 0.04 m 全开；`right_finger_joint` 是 mimic，只出现在 TF），以及 action
`<ns>/gripper_controller/gripper_cmd`（`control_msgs/action/ParallelGripperCommand`）。mock 由
`gripper_controller`（`parallel_gripper_action_controller`）提供；真机由驱动插件自己提供同名同类型的 action，
不加载 `gripper_controller`——上游控制器只看开度速度，分不清夹住物体与等机械臂，且中止后目标仍留在命令接口里。

真机 action 的语义：

- **结果**：`motion_done` 是一次运动内锁存的两个标志位，0x01 到达目标、0x02 途中受阻（G13）。运动结束 = 帧报过 0
  之后，标志位非 0、且标志位与开度至少 300 ms 未变（新运动开始后帧里会短暂再现上一次的标志位，G17），目标不为 0 %
  时开度也不为 0 %。1 到位：SUCCEEDED（`reached_goal`）；2 夹住物体：SUCCEEDED 且 `stalled = true`；3 先受阻后到达，
  即运动中失去了物体：ABORTED；其他值同样 ABORTED。发出后 1.5 s 未见运动开始、或 30 s 未完成：ABORTED。
  结果里的 `state.position` 为结束时的开度。目标为 0 % 时不做额外防护：若旧标志位和 0 % 开度持续超过 300 ms，
  可能提前判定结束；这是已知局限，真机尚未观察到
- **夹住之后不可观测**：标志位仅在命令的 `max_time` 内改变，超时后物体掉落、被取走，帧仍可报 2（G13）；驱动没有别的传感器，
  不报告"是否仍夹着"
- **等待**：机械臂在动时目标等它停下，不限时（调用方可取消）
- **一次一个目标**：新目标被接受时旧目标立即 ABORTED；旧目标已发出的运动走完后才发新目标（控制器不能打断夹爪运动，G15）
- **取消**：尚未发出的目标取消后永不执行；已发出的立即报 CANCELED，运动走完
- **拒绝与作废**：硬件未激活、夹爪未激活或报故障时拒绝新目标；等待中或运动中的目标遇到硬件停用、机器人故障、
  夹爪不可用或激活夹爪即 ABORTED，恢复后也不执行
- 到当前开度的目标照发（可用于再次夹紧）
- mock 的上游控制器按开度速度判断卡住（`stall_timeout` 2 s），语义与真机不同

```bash
ros2 launch fr_robot_driver fr_control.launch.py model:=fairino5_v6 gripper:=pgi140 [use_mock_hardware:=false ip:=<控制器>]
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand \
  "{command: {name: [left_finger_joint], position: [0.02]}}"          # 开到 50 %
```

**安全约定（真机）**：

- **激活只由操作者发起**：`ros2 service call /<ns>/<tf_prefix>fr_robot_hw/gripper/activate std_srvs/srv/Trigger`。
  激活必然张开到满行程，不论手里有什么（sdk_quirks G2）：调用前确认手指之间无物、张开路径无障碍。
  驱动在启动、configure、activate、复位时都不激活。激活状态属于控制器，每次控制箱上电一次（G1）。
  服务在已激活、机械臂故障、机械臂在动（任一关节 > 0.5 °/s，或伺服流未停发）、硬件未 configure 时拒绝；
  夹爪故障不拒绝——激活就是清除夹爪故障的办法（G11）。否则发 `ActGripper(index, 1)`，等状态帧报激活且无故障
  （最多 3 s）后返回；激活期间机械臂保持不动

**夹爪与机械臂轮流动**（控制器的限制，G8、G9）：伺服流（`ServoJ`）进行中控制器压住 `MoveGripper` 不执行，
夹爪运动中又压住 `ServoJ` 不执行。所以：

- **机械臂静止时停发**：要发的点与上一条逐字节相同、且回读已到达（各轴 ≤ 0.01°），伺服流就停发
  （不调 `ServoMoveEnd`，`~/status` 的 `servo_stream = paused`、`servo_queue_len` 为 0）；
  下一条不同的命令先补 `prefill` 段原地不动的再续发，从静止起步多约 `prefill × cmdT`（24 ms）延迟（S18）
- **夹爪目标等机械臂静止**：伺服流已停发且控制器队列为空时才发 `MoveGripper`；机械臂在动或同一周期要动时，
  目标等着
- **夹爪忙时轨迹暂停（误用兜底）**：从发出 `MoveGripper`（或激活）到确认完成（标志位与开度稳定，见上文；1.5 s 内
  未见运动开始或 30 s 未完成则目标 ABORTED 并放行机械臂），`speed_scaling_factor` 为 0，`joint_trajectory_controller` 的轨迹时间停住：
  其间收到的轨迹停在起点等待，夹爪完成后从原处继续并照常 SUCCEEDED，不触发路径容差，**硬件不停用**
  （`~/status` 的 `arm_commands = paused (gripper busy)`）。驱动也不把会让机械臂动的命令发给控制器（控制器在夹爪
  运动期间不执行 `ServoJ`，G9）；夹爪完成的那个周期起命令照常发出，只经单周期步长检查。`MoveGripper` 调用失败
  （返回错误或超时）也一样只看状态帧：超时的调用可能在伺服流停下后照样执行（G8）。
  不认速度缩放的控制器（如 `forward_position_controller`）在夹爪运动期间照样往前走，夹爪完成后命令一步跳过去，
  被步长检查拒绝、保护停止：夹爪运动期间不要用它们命令机械臂。

**使用约定**：机械臂与夹爪顺序执行，等前一个 action 结束再发下一个；夹爪运动时不应有机械臂运动。轨迹暂停与夹爪
等待只作误用时的兜底。MoveIt 的执行时长监控会把等待时间算进去，接入时须为夹爪控制器配置足够的时长。

**命令与开度**：

- 每个目标发一次 `MoveGripper`（`block = 1`；`block = 0` 不阻塞且会让夹爪失去激活、报故障，G11），
  由后台线程用独立的 20003 连接发送；`read()`/`write()` 不碰网络。PGI-140 的速度参数无效（G12），goal 的 `velocity`
  被忽略，协议速度固定发 50 %，无 `max_velocity` 命令接口。力取自 goal 的 `effort`：力 % = N ÷ 1.4（满量程 140 N），
  限制在 1–100；未给有效正数时用 20 %（28 N，夹玻璃器皿），与 mock 的 `gripper_controller.max_effort` 一致
- 开度 = 状态帧的 `gripper_position` %（整数）× 0.04 m。控制器与夹爪之间通信慢：每次运动开始约 0.8 s 内帧报 0 %、
  未完成，之后约每 0.4 s 才更新一次（G10）；运动到当前开度时，完成后帧还会报 0 % 约 200 ms（G14）。帧报"0 %、未完成"时，
  以及目标不为 0 % 而发出后帧还没报过非 0 开度时，沿用上一个已知开度
  （`~/status` 的 `gripper_position_source = held_during_motion`）；激活后（必然全开）或驱动启动时还不知道开度，
  则报全开（`inferred_open_after_activation`）；其余为 `frame`。**未激活时**帧报 0 %，与手指实际位置无关，开度不可知：
  驱动报全开（外轮廓最大，碰撞检查偏保守；`unknown_not_active`），激活（必然张开到满行程）后即为真。
  **上层不要把“报全开”当作实测开度**：`gripper_position_source = frame` 只表示开度来自状态帧，也不能证明物体仍被夹住
- 夹爪速度状态恒报 0；帧更新稀疏，不估计速度。完成判定与开度推断由纯逻辑 `GripperMotion` 处理，
  `test_gripper_motion` 回放真机帧序列；假控制器不模拟这些固件怪癖
- 夹爪编号 `gripper_index`（launch 参数，默认 1）；编号 N 对应状态帧 `gripper_active` 的 bit N−1（G5）

`~/status` 另有 `servo_stream`、`arm_commands`，以及 `gripper_active`、`gripper_fault`、`gripper_motion_done`、
`gripper_busy`、`gripper_position`（%）、
`gripper_command_failures`（失败的 `MoveGripper` 次数，失败原因记在日志里）。

## 用法

```bash
# 单机（默认 fairino20_v6，mock）
ros2 launch fr_robot_driver fr_control.launch.py

# 指定型号 + tf_prefix + 命名空间
ros2 launch fr_robot_driver fr_control.launch.py \
  model:=fairino3_v6 namespace:=alice tf_prefix:=alice_

# 双机工作台 demo（默认 fairino5_v6，两臂 + 1.2m×1.8m 台面 + RViz）
ros2 launch fr_robot_driver dual_fr_cell.launch.py
```

## 双机工作台 demo

两台 FR5 对称安装（x=±0.4m，相向），演示多机隔离与初始化配置的重要性：

```bash
# 纯 FK 碰撞分析（无需 ROS 图）
ros2 run fr_robot_driver zero_pose_collision_check.py fairino5_v6
```

- **零位**：两臂水平伸直越过中线，恰好穿过对方肩柱（FK 间隙 -0.17m）
- **经典 ready 位** `[0,-90°,+90°]`：小臂上折后沿中线相向，尖端仅距 1cm——
  单机安全的位形在多机场景**并不安全**
- **工作台位** `[pan+90°,-90°,+90°]`（`workbench_initial_positions.yaml`）：
  两臂折向各自台端，间隙 +0.60m ✓

教训：**初始位形是场景的属性，不是机器人的属性**——`initial_positions_file`
由单元级 launch 按场景传入。同时注意 ros2_control 本身不做碰撞检查
（mock 会毫无阻拦地执行互穿轨迹），多机安全需要规划层（MoveIt）保证。

## RViz 配置在哪里

| 场景 | 配置文件 | 由谁加载 |
|---|---|---|
| 双机工作台 | `fr_robot_driver/rviz/dual_cell.rviz` | `dual_fr_cell.launch.py`（`launch_rviz` 参数，默认 true） |
| 单机可视化 | `fr_description/rviz/view_robot.rviz` | `fr_description/launch/view_robot.launch.py` |

安装后位于 `share/<包>/rviz/`，也可手动打开：

```bash
# 双机工作台（需先启动 dual_fr_cell，或手动打开对应话题）
rviz2 -d $(ros2 pkg prefix fr_robot_driver)/share/fr_robot_driver/rviz/dual_cell.rviz
```

`dual_cell.rviz` 的结构（Fixed Frame = `world`）：

- **Grid** — 台面坐标参考
- **TF** — 全帧关闭（减噪）+ **TF Key Frames** 只显示关键帧：
  `world`、`alice_/bob_` 的 `base_link`/`flange`/`tool0`
- **RobotModel ×3** — 各自订阅对应命名空间的描述话题：
  `/alice/robot_description`、`/bob/robot_description`、`/table/robot_description`
  （每个机器人一个 robot_state_publisher，描述话题即隔离接口）
- 预置视角（Views 面板）：*Overview (oblique)*、
  *Zero-pose check (closeup)*（零位互碰观察位）、
  *Flange closeup (both)*（中线对称特写）

演示零位互碰：启动后把两臂发到零位（`ros2 action send_goal ...`），
切到 *Zero-pose check* 视角即可看到两臂精确穿过对方肩柱。

## 多机隔离机制

| 维度 | 隔离手段 |
|---|---|
| topic/action | 节点 namespace（`/alice/joint_states`、`/bob/...`） |
| TF frame | `tf_prefix`（`alice_base_link`、`bob_tool0`），由 URDF 层保证 |
| controller_manager | 每机一个，namespaced |
| 控制器参数 | `config/controllers.yaml` 的 `__PREFIX__`/`__NS__` 令牌由 launch 按机替换 |

## 文件结构

```
urdf/fr_controlled.urdf.xacro       顶层：fr_description 宏 + 可选夹爪宏 + fr_ros2_control 宏
urdf/fr_ros2_control.xacro          <ros2_control> 生成宏（joint 名带 tf_prefix；mock 与真机同名 fr_status、speed_scaling gpio）
src/ include/ fr_robot_driver.xml   硬件插件 FrRobotHardwareInterface（+ reset_policy.hpp）
test/                              插件状态机测试（假控制器 + ResourceManager）
urdf/table_cell.urdf.xacro          工作台静态场景（world + 台面）
config/controllers.yaml             控制器配置（令牌模板）
config/initial_positions.yaml       单机初始位形
config/workbench_initial_positions.yaml  双机工作台初始位形
launch/fr_control.launch.py         单机 bringup（可被单元 launch 多次 include）
launch/dual_fr_cell.launch.py       双机工作台（含台面 rsp + RViz）
rviz/dual_cell.rviz                 工作台可视化配置
scripts/zero_pose_collision_check.py 纯 FK 零位碰撞分析
```

## 踩坑记录（本环境 Jazzy / ros2_control 4.48.1，源码在 `references/ros2_control`）

- `<ros2_control type=...>` 必须小写 `"system"`：大小写比较敏感，写错会静默
  跳过组件并报误导性的 "Wrong state or command interface configuration"
- 初始值参数键是 `initial_value`（旧键 `initial_position` 被静默忽略）
- Humble 时代需要自行再注册 mock 插件 XML；Jazzy 已由 ros2_control 正常导出，
  该 workaround 已随迁移删除
- 新版 spawner 的 `--param-file` 要求根键为**带命名空间的控制器全名**，
  且控制器必须声明插件 `type`
