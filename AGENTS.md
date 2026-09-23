# AGENTS.md

本仓库的开发约定，供 coding agent 和开发者遵循。

## 核心规则

### 1. `references/fairino_vendor/` 是外部基线，禁止修改、禁止在 src 中引用

来自上游 GitHub 仓库 **[frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2)**
（官方，许可未明确），仅作只读参考：

- `fairino_description/` — vendored URDF/mesh（数据来源基线）
- `fairino*_moveit2_config/` — MoveIt Setup Assistant 生成物（配置基线）
- `fairino_flange_description/` — 已废弃的 wrapper 方案（被 fr_description 取代）

修改一律落在 `src/` 内我们自己的包中。

### 2. `src/` 中的 `fr_description` 是我们自己维护的描述包

对标 `ur_description` 的宏 + 参数文件架构（详见包内 README.md）：

- **config/*.yaml 和 urdf/fr_macro.xacro 是源头**，可以直接修改；
- 不要为了改参数去碰 `references/fairino_vendor/`；
- 新增型号：准备该型号的 4 个参数 YAML + mesh 目录（可参考
  `scripts/generate_from_vendor.py` 的输出格式），macro 无需改动。

### 3. `src/` 中的 `fr_robot_driver` 是 ros2_control 驱动包（mock 优先）

对标 `ur_robot_driver`，渐进式开发：**在 `use_mock_hardware:=true` 管线
全绿之前不碰真机 SDK**（`fr_robot_driver/FrRobotHardwareInterface` 占位中）：

- 初始位形是**场景属性**：单机用 `config/initial_positions.yaml`，
  多机单元必须传入场景专属初始位形（如 `workbench_initial_positions.yaml`），
  修改布局后用 `zero_pose_collision_check.py` 复核；
- 本环境 ros2_control 为 2.54.2（源码在 `references/ros2_control`），
  遵循其新约定：`<ros2_control type="system">` 小写、`initial_value` 键、
  spawner 参数文件根键带命名空间且必须声明 `type`（详见包内 README 踩坑记录）；
- 验证命令：launch 后 `ros2 control list_controllers` 应为 active，
  `ros2 topic echo /<ns>/joint_states --once` 应有初始位形值，
  发 FollowJointTrajectory 后读回轨迹终点。

### 参考：`references/` 还有两份重要基线

- `references/ros2_control/` — 本环境安装的 ros2_control 2.54.2 源码
  （行为与老版 Humble 差异较大，遇疑难报错先查它）
- `references/Universal_Robots_ROS2_Tutorials/` — 架构范本

## 相关文档

- [fr_description/README.md](fr_description/README.md) — 架构、用法、命名规范化对照表
- [fr_robot_driver/README.md](fr_robot_driver/README.md) — 驱动架构、双机工作台 demo、踩坑记录
- [docs/extension_pattern.md](docs/extension_pattern.md) — 扩展方式与验证命令
- [docs/vendor_urdf_issues.md](docs/vendor_urdf_issues.md) — vendor 已知问题历史（已全部被新描述包吸收）
- [docs/robot_dh_params.md](docs/robot_dh_params.md) — 各型号 D-H 参数（flange_offset 以此为准）
