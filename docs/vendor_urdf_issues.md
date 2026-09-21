# Vendor URDF 已知问题（迁移记录）

> **状态（已全部解决）**：以下问题来自 vendored URDF（上游
> [frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2)），已全部被
> `fr_description` 的命名规范化吸收——vendor 文件仅存于
> `references/fairino_vendor/` 作只读基线。本文保留作迁移记录与对照。

| vendor 问题 | 涉及文件 | 新包中的处理 |
|---|---|---|
| `tool_Link`/`gripper_Link` 的 link 定义被注释掉，但 `tool` joint 仍引用（断链，URDF 无法解析） | `FR3WML.urdf`、`FR3WMS.urdf`、`FR5WM.urdf` | 断链 joint 丢弃；`flange` + `tool0` 取代其语义 |
| link 名大小写不一致：FR* 系列用 `wrist3_Link`（大写 L），v6 系列用 `wrist3_link` | 全部 FR*.urdf | 统一为 `wrist_3_link` 等规范名 |
| 关节名两套：`j1..j6` 与 `joint1..joint6`（fairino3_mt_v6） | 混用 | 统一为 `shoulder_pan_joint … wrist_3_joint` |
| 拼写错误 `uppperarm_link`（三个 p） | `fairino30_v6.urdf` | 统一为 `upper_arm_link` |
| robot name 缺下划线：`fairino3mt_v6_robot` | `fairino3_mt_v6.urdf` | 顶层 `<robot name>` 由消费方决定 |
| 全部 link 缺 `<collision>` | `fairino3_mt_v6.urdf` | 保留现状（YAML 中 `mesh: null`，宏按需省略） |
| FR* 系列引用相对 mesh 路径 `../meshes/...`（URDF 加载期无法解析） | 全部 FR*.urdf | 重写为 `package://fr_description/meshes/...` |
| 零值 `<dynamics>`/`<calibration>`/`<safety_controller>`（SolidWorks 导出噪音） | 全部 | 丢弃 |

## 背景说明

- FR* 系列（`FR3WML/FR3WMS/FR5WM.urdf`）是 SolidWorks 导出版，v6 系列是厂商整理版，
  两者出自不同流水线，命名风格不一致是历史原因。
- 迁移时的数值等价性（关节原点/轴/限位、质量/质心/惯量、mesh 路径）已逐项脚本化校验，
  见 `fr_description/README.md`。
