# fr_robot_driver

ros2_control bringup for Fairino 协作机器人，对标 `ur_robot_driver`（见
`references/Universal_Robots_ROS2_Tutorials` 的 `my_robot_cell_control` 模式）。

## 两条管线

1. **mock**：`use_mock_hardware:=true`（默认），`mock_components/GenericSystem`。
   `joint_state_broadcaster` + `joint_trajectory_controller` 完整可用。
2. **真机 / 仿真器**：`use_mock_hardware:=false`，`fr_robot_driver/FrRobotHardwareInterface`
   （C++ 插件，依赖 `fr_client_library`，软件版本 3.8.4；3.8.3 仿真器已验证）。
   两条管线暴露**同样的** `fr_status` 状态接口，上层不必区分。

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
| 固件故障 / 碰撞 / 安全停止 / 掉使能 | 立即停发，硬件 → `inactive`，`fr_status/fault_class = 1`，故障码原样导出 | 排除原因 → `reset_error` → 激活硬件 → 激活控制器 |
| 命令超过单周期步长（URDF 速度限位 × cmdT） | 拒绝该命令（固件从未见到），同上，`main_code` 仍为 0 | 同上 |
| 急停 | 同上，`fault_class = 2`；`reset_error` **拒绝** | 只能硬件重启控制器，之后重新 configure |
| 通信中断（状态帧超时/连接断开/ServoJ 连接失败） | 硬件 → `unconfigured`，`fault_class = 3`；`reset_error` 拒绝 | 确认控制器在线后重新 configure → activate |

```bash
ros2 service call /<ns>/<tf_prefix>fr_robot_hw/reset_error std_srvs/srv/Trigger   # 例：/fr_robot_hw/reset_error
ros2 control set_hardware_component_state <tf_prefix>fr_robot active
ros2 control set_controller_state joint_trajectory_controller active
```

`fr_status` 接口（`<tf_prefix>fr_status/...`，经 `joint_state_broadcaster` 发布在 `dynamic_joint_states`）：
`fault_class`、`estop`、`safety_stop0/1`、`collision`、`main_code`、`sub_code`、`enabled`、`robot_mode`、
`robot_state`、`servo_queue_len`（固件 ServoJ 队列占用；时钟偏差补偿使它稳态在 `prefill` 到 `prefill + 1` 之间）。

### 注意

- **同一控制器上 20004 和 8080 都只服务一个客户端**：驱动运行时，`fr_probe`、`fr_servo_bench` 等工具收不到状态帧。
- 控制器管理器的 `update_rate` 必须满足 `update_rate × cmd_t_ms / 1000` 为 ≥ 1 的整数，否则插件拒绝初始化；
  更快的控制周期按整数倍抽取（500 Hz、8 ms → 每 4 个周期发 1 条）。
- `ServoJ` 不受全局速度/加速度限制（sdk_quirks S14）：保护靠 URDF 限位、`enforce_command_limits`（真机分支默认开）
  和插件的步长检查。
- mock 分支加 `mock_gpio_commands:=true` 会给 `fr_status` 加回环命令接口，用来演练上层的故障处理。

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
urdf/fr_controlled.urdf.xacro       顶层：fr_description 宏 + fr_ros2_control 宏
urdf/fr_ros2_control.xacro          <ros2_control> 生成宏（joint 名带 tf_prefix；mock 与真机同名 fr_status）
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
