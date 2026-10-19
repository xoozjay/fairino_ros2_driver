# AGENTS.md

开始任何工作前，若技能列表里有 `kcocoa-project-workflow`，先读它。

## 项目

面向法奥（Fairino）协作机器人的 ROS 2 包集合，对标 UR 官方 ROS 2 驱动与教程：参数化描述包 `fr_description`、
ros2_control 驱动 `fr_robot_driver`、自研 C++ 客户端库 `fr_client_library`（不链接厂商 SDK）、`fr_moveit_config`，
支持多机械臂单元（各自命名空间 + `tf_prefix` + 独立驱动）。

不在范围内：本机路径、账号、凭证、维护者私有流程——这些不得写入本仓库。

## 常用命令

- **Check（每次提交前必须通过）：`./scripts/check.sh`** —— 构建本仓库全部包并运行 gtest；
  从零验证同一工作区用 `rm -rf build install log` 后再跑同一条命令
- Build：工作区根目录 `colcon build`（默认 symlink-install + ccache + compile_commands，
  见 `/etc/colcon/defaults.yaml`）；单包 `colcon build --packages-up-to fr_robot_driver`
- Run：`ros2 launch fr_robot_driver fr_control.launch.py`（默认 mock）；
  仿真器/真机加 `use_mock_hardware:=false ip:=$FR_ROBOT_IP`
- Logs：`log/latest_test/`、`build/<pkg>/Testing/Temporary/LastTest.log`、`~/.ros/log/`
- Environment：容器内工作区为 `/workspace/chemistry_os2_ws`；宿主用 `./dev shell`、`./dev run <cmd>`。
  控制器（真机，或仿真器 `192.168.58.2`）的 20004 与 8080 端口同时只服务一个客户端，驱动运行时 `fr_probe` 等工具连不上

## Project rules

- `references/fairino_vendor/` 只读，`src/` 不得引用；改描述参数改 `fr_description/config/*.yaml`
- 版本一律以 3.8.4 为准（兼容下限 3.7.6），不借用更新版本的 API 与行为；版本以 `GetSoftwareVersion` 的
  `webVersion` 为准。锁定表与 `references/` 恢复命令见 [docs/sdk_quirks.md](docs/sdk_quirks.md) §一
- 写硬件插件前读 [docs/sdk_quirks.md](docs/sdk_quirks.md) 与 [docs/fault_handling.md](docs/fault_handling.md)：
  `read()`/`write()` 内不调 SDK；`on_deactivate`/`on_error` 在实时线程内同步调用，不做网络操作（交给清理线程）；
  故障期间停发运动指令，绝不自动复位
- 指令的通道、方法、参数与帧内容按 SDK 3.8.4 原样实现（传输层健壮性改进除外）
- 只引用公开资料（官方文档、SDK 源码）与仿真器/真机黑盒实测；非公开资料的内容、路径或出处不得写入文件或提交信息
- 外部系统（控制器、SDK、夹爪）的行为写进 `docs/sdk_quirks.md`，每条带来源标记
- 实验脚本与数据不入本仓库（含现场点位）；结论写进 `docs/` 时注明来源与日期范围
- 控制器地址取 `$FR_ROBOT_IP`，未设置就问；不把真机地址写进任何文件
- 初始位形是场景属性：单机用 `config/initial_positions.yaml`，多机单元传入场景专属文件，改布局后用
  `zero_pose_collision_check.py` 复核
- 实时状态（起始位形、夹爪开度、是否激活）每次用 `fr_probe` 读，不记录
- **真机操作**：运动前向维护者确认有人在急停旁与本次允许的运动范围；控制器访问一律经
  `flock /tmp/fr_robot_<ip>.lock`；不发夹爪指令、不写负载/工具、不使能/下使能、不复位——这些只由操作者发起
  （sdk_quirks F3、F6、G2）；急停会让控制箱断电，之后须断电重启（F4）；结束进程按 PID，`pkill -f` 会匹配到自己的 shell

## 文档地图

| 文件 | 内容 |
|---|---|
| [TODO.md](TODO.md) | 未决事项、未验证的断言、已知风险 |
| [docs/plans/](docs/plans/) | 大改动的计划 |
| [docs/architecture.md](docs/architecture.md) | 架构图（Mermaid）：包、ROS 图、插件结构、线程模型、伺服链路、夹爪与启动时序 |
| [docs/sdk_quirks.md](docs/sdk_quirks.md) | 版本锁定；SDK 与固件（3.8.4）陷阱清单与驱动设计约束 |
| [docs/fault_handling.md](docs/fault_handling.md) | 急停/故障分级、状态机与恢复流程 |
| [docs/extension_pattern.md](docs/extension_pattern.md) | 扩展方式与验证命令 |
| [docs/robot_dh_params.md](docs/robot_dh_params.md) | 各型号 D-H 参数（flange_offset 以此为准） |
| [docs/vendor_urdf_issues.md](docs/vendor_urdf_issues.md) | vendor URDF 已知问题（已被 fr_description 吸收） |
| [fr_description/README.md](fr_description/README.md) | 描述包架构、用法、新增型号、命名规范化 |
| [fr_client_library/README.md](fr_client_library/README.md) | 客户端库的三条通路、设计决策、工具与测试 |
| [fr_robot_driver/README.md](fr_robot_driver/README.md) | 驱动架构、验证、故障与恢复、双机工作台、Jazzy 踩坑 |
| `references/fairino_vendor/` | 上游 [frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2) 的 URDF/mesh 与 MoveIt 配置，只读基线 |
| `references/ros2_control/` | 本环境 ros2_control 4.48.1 源码，疑难报错先查它 |
| `references/Universal_Robots_ROS2_Tutorials/` | 架构范本 |
