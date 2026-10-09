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
- 本环境为 ROS 2 Jazzy，ros2_control 为 4.48.1（源码在 `references/ros2_control`），
  遵循其新约定：`<ros2_control type="system">` 小写、`initial_value` 键、
  spawner 参数文件根键带命名空间且必须声明 `type`（详见包内 README 踩坑记录）；
- 验证命令：launch 后 `ros2 control list_controllers` 应为 active，
  `ros2 topic echo /<ns>/joint_states --once` 应有初始位形值，
  发 FollowJointTrajectory 后读回轨迹终点。

### 4. 版本约束（硬性）

目标固件 **v3.8.4**，兼容下限 **3.7.6**（不支持更早版本）。凡随固件版本变化的内容（API、参数形状、端口、帧格式、行为），一律以 3.8.4 为准，
**不得**参考或借用更新版本的 API 与行为（如 3.9.x 的 UDP 20007 伺服、CNDE 20005 订阅、`comType` 参数）。

| 对象 | 锁定版本 | 位置 |
|---|---|---|
| 控制器固件 | **v3.8.4** | 真机 |
| C++ SDK（裁剪自 [fairino-cpp-sdk](https://github.com/FAIR-INNOVATION/fairino-cpp-sdk)，首选阅读） | 分支 `3.8.4` = `831c662`（同 tag `v2.2.4-robot3.8.4`，`libfairino.so.2.2.4`） | `references/libfairino` |
| C++ SDK（上游原始） | tag `v2.2.4-robot3.8.4` = `9543a2d` | `references/fairino-cpp-sdk` |
| 官方文档 | tag `3.8.4` = `f34272d` | `references/fr_documentation` |
| 仿真器 | 官方 SimMachine（Docker）v3.8.3，本机 `192.168.58.2`（自报 `V3.8.8-LA` / web `v3.8.3`），只做功能/协议验证，时序数据无效 | — |
| ROS 2 / ros2_control | Jazzy / 4.48.1 | `references/ros2_control` |

- 阅读 `references/` 前先确认版本：`git -C references/<repo> describe --tags --always`。
  恢复命令（在工作区根目录执行）：

  ```sh
  git -C references/libfairino switch 3.8.4
  git -C references/fairino-cpp-sdk switch --detach v2.2.4-robot3.8.4
  GIT_LFS_SKIP_SMUDGE=1 git -C references/fr_documentation switch --detach 3.8.4
  ```

- `references/libfairino/docs/`（未跟踪）是早期针对 3.7.8 / 3.9.8 写的分析，与 3.8.4 源码冲突时以源码为准；
- 3.8.3 仿真器上通过 ≠ 3.8.4 真机通过（SDK 3.8.3 → 3.8.4 差异见 [docs/sdk_quirks.md](docs/sdk_quirks.md)）；
- **版本以 `GetSoftwareVersion` 返回的软件版本为准**（第二个字符串 `webVersion`，如 `v3.8.4`；
  本文"固件版本"即指它）。第三个字符串 `controllerVersion`（如 `V3.8.8-LA`）是厂商的另一套编号，只显示、不比较；
  厂商的 `GetFirmwareVersion` 指控制箱板/驱动器固件，与此无关；
- 驱动按软件版本做真机验证白名单（当前 3.8.3 仿真器、3.8.4 目标；`software_version.hpp`）。
  真机报 **3.8.4.2**：第四位及以后是 bug 修复版本、不改功能，**视为同一版本**（解析时忽略，不做过滤）。仿真器只有 3.8.3，没有 3.8.4；
  状态帧长也由软件版本映射，**不靠帧长猜版本**。白名单之外的版本协议层大概率能用，但未经验证，不放行；
- 升级版本 = 上表全部重新锁定 + 重新审计 sdk_quirks.md。

### 5. 真机 SDK 与故障处理：先读文档

- 写硬件插件前必读 [docs/sdk_quirks.md](docs/sdk_quirks.md)（每次调用新建 XML-RPC
  连接、无超时、状态撕裂/清零、cmdT 取整到 1 ms、8080 单客户端等）；`read()`/`write()` 内禁止调用 SDK；
- 指令的通道、方法、参数与帧内容**按 SDK 3.8.4 原样实现，不做修改**（传输层健壮性改进除外）；
- 本仓库只引用**公开资料**（官方文档、SDK 源码）与**仿真器/真机黑盒实测**；任何非公开资料的内容、
  路径或出处都不得写入本仓库的文件或提交信息；
- 故障处理遵循 [docs/fault_handling.md](docs/fault_handling.md)：故障期间停发运动指令、**绝不自动复位**。

### 参考：`references/` 中的其他基线

- `references/ros2_control/` — 本环境安装的 ros2_control 4.48.1 源码（Jazzy，
  tag `4.48.1`），遇疑难报错先查它
- `references/Universal_Robots_ROS2_Tutorials/` — 架构范本

## 相关文档

- [fr_description/README.md](fr_description/README.md) — 架构、用法、命名规范化对照表
- [fr_robot_driver/README.md](fr_robot_driver/README.md) — 驱动架构、双机工作台 demo、踩坑记录
- [docs/extension_pattern.md](docs/extension_pattern.md) — 扩展方式与验证命令
- [docs/sdk_quirks.md](docs/sdk_quirks.md) — libfairino SDK（固件 3.8.4）陷阱清单与驱动设计约束
- [docs/fault_handling.md](docs/fault_handling.md) — 急停/故障分级、状态机与恢复流程（固件硬性要求）
- [docs/vendor_urdf_issues.md](docs/vendor_urdf_issues.md) — vendor 已知问题历史（已全部被新描述包吸收）
- [docs/robot_dh_params.md](docs/robot_dh_params.md) — 各型号 D-H 参数（flange_offset 以此为准）
