# fr_description

法奥（Fairino）协作机器人的参数化描述包，设计对标 `ur_description`：**一个宏 +
按型号的参数文件** 生成运动学树，而不是为每个型号维护一份成品 URDF。

## 设计

```
urdf/fr_macro.xacro     宏 fr_robot：生成整棵树
                             ├─ 6 个转动关节（语义化命名，对齐 ur_description）
                             ├─ 7 个连杆 + flange（法兰面，DH d6）+ tool0（工具挂载）
                             └─ 参数：parent / origin_xyz / origin_rpy / tf_prefix
config/<model>/              每型号 4 个参数文件
                             ├─ default_kinematics.yaml   关节原点/轴 + flange_offset
                             ├─ joint_limits.yaml         关节限位
                             ├─ physical_parameters.yaml  质量/质心/惯量
                             └─ visual_parameters.yaml    可视/碰撞 mesh
meshes/<model>/              各型号 mesh（来自 vendor）
launch/view_robot.launch.py  RViz 可视化（model/tf_prefix 参数）
scripts/generate_from_vendor.py  一次性迁移工具（见下）
```

## 使用

```bash
# 可视化任一型号
ros2 launch fr_description view_robot.launch.py model:=fairino20_v6

# 在自己的描述/控制包中组装（推荐，可传 tf_prefix）
<xacro:include filename="$(find fr_description)/urdf/fr_macro.xacro"/>
<xacro:fr_robot
    parent="world" tf_prefix="alice_"
    kinematics_parameters_file="$(find fr_description)/config/fairino20_v6/default_kinematics.yaml"
    joint_limits_parameters_file="$(find fr_description)/config/fairino20_v6/joint_limits.yaml"
    physical_parameters_file="$(find fr_description)/config/fairino20_v6/physical_parameters.yaml"
    visual_parameters_file="$(find fr_description)/config/fairino20_v6/visual_parameters.yaml"/>
```

`tf_prefix` 使多台机器人可共存于同一 URDF/TF 树（双臂单元），静态 vendor URDF 做不到这一点。

## 支持型号

fairino3_v6, fairino3_mt_v6, fairino5_v6, fairino10_v6, fairino16_v6,
fairino20_v6, fairino30_v6, fr3wml, fr3wms, fr5wm

新增型号：在 `config/<model>/` 放 4 个参数文件、在 `meshes/<model>/` 放 mesh（格式同
`scripts/generate_from_vendor.py` 的输出），宏无需改动。

`config/*.yaml` 与 `urdf/fr_macro.xacro` 是数据源头，改参数直接改它们；`references/fairino_vendor/`
只是提取时的只读基线。

## 数据来源与命名规范化

参数文件与 mesh 提取自 vendored URDF（上游
[frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2)，官方未明确协议），
存于 `references/fairino_vendor/`，提取时做了命名规范化：

| vendor 原名 | 规范名 |
|---|---|
| `j1..j6` / `joint1..joint6` | `shoulder_pan_joint` … `wrist_3_joint` |
| `upperarm_link` / `uppperarm_link`(拼写错误) / `upperarm_Link` | `upper_arm_link` |
| `wrist3_Link`（FR* 系列大写） | `wrist_3_link` |
| 断链的 `tool_Link`/`gripper_Link`（FR* 系列） | 由 `flange` + `tool0` 取代 |

数值（关节原点、限位、质量、惯量、mesh 路径）与 vendor 完全等价，已脚本化逐项校验。
重新生成：`scripts/generate_from_vendor.py --help`。

## 许可

AGPL-3.0（与仓库根 LICENSE 一致）。提取自上游的数据，其许可状态见仓库 README 的说明。
