# fr_robot_driver

ros2_control bringup for Fairino 协作机器人，对标 `ur_robot_driver`（见
`references/Universal_Robots_ROS2_Tutorials` 的 `my_robot_cell_control` 模式）。

## 渐进式开发

1. **当前阶段（mock）**：`use_mock_hardware:=true`（默认），使用
   `mock_components/GenericSystem`。控制器管线完整可用：
   `joint_state_broadcaster` + `joint_trajectory_controller`，端到端可验证。
2. **下一阶段（真机）**：`use_mock_hardware:=false` 切换到
   `fr_robot_driver/FrRobotHardwareInterface`（占位中，插件待实现）。

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
urdf/fr_ros2_control.xacro          <ros2_control> 生成宏（joint 名带 tf_prefix）
urdf/table_cell.urdf.xacro          工作台静态场景（world + 台面）
config/controllers.yaml             控制器配置（令牌模板）
config/initial_positions.yaml       单机初始位形
config/workbench_initial_positions.yaml  双机工作台初始位形
launch/fr_control.launch.py         单机 bringup（可被单元 launch 多次 include）
launch/dual_fr_cell.launch.py       双机工作台（含台面 rsp + RViz）
rviz/dual_cell.rviz                 工作台可视化配置
scripts/zero_pose_collision_check.py 纯 FK 零位碰撞分析
```

## 踩坑记录（本环境 ros2_control 2.54.2，源码在 `references/ros2_control`）

- `<ros2_control type=...>` 必须小写 `"system"`：大小写比较敏感，写错会静默
  跳过组件并报误导性的 "Wrong state or command interface configuration"
- 初始值参数键是 `initial_value`（旧键 `initial_position` 被静默忽略）
- 该发行版未导出 `mock_components_plugin_description.xml`：本包通过
  package.xml `<hardware_interface plugin=.../>` 再注册（见
  `mock_components_plugin_description.xml` 文件头注释）
- 新版 spawner 的 `--param-file` 要求根键为**带命名空间的控制器全名**，
  且控制器必须声明插件 `type`
