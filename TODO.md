# TODO

未决事项、未验证的断言与已知风险。设计决策与现状见 [AGENTS.md](AGENTS.md)、各包 README 和
`docs/`；改动历史见 `git log`。

## 待办

### 规划层（fr_moveit_config）

- [ ] 对 `fr_description` 跑 MoveIt Setup Assistant（tip 选 `flange`），替代
      `references/fairino_vendor/` 里的 7 份生成物
- [ ] SRDF / kinematics / controllers 对接新命名（语义关节名）
- [ ] 多机规划场景（双臂互碰检查）——ros2_control 不查碰撞，工作台 demo 的互穿轨迹要靠它拒绝

### 单元组合层

- [ ] 工作台场景升级为完整单元（对标 `my_robot_cell` / `my_dual_robot_cell`）
- [ ] 评估 Gazebo / Ignition 仿真路线（当前只有 mock）

### 杂项

- [ ] 本仓库尚未配置远程；首次 push 需 `git lfs push --all` 同步 mesh 对象（96 个 LFS 文件）
- [ ] `initial_positions.yaml`（单机默认位形）与工作台位形的关系可考虑写进
      [docs/extension_pattern.md](docs/extension_pattern.md)

## 未验证

以下行为只在 3.8.3 仿真器上观察过；**仿真器时序数据无效**，真机取值待确认：

- 状态推送周期最小值（仿真器 8 ms，≤ 7 ms 被拒绝并返回码 4）
- 8080 即时指令回执内容 `1` 的含义与失败取值；收发超时默认 1000 ms
- 参数个数不符时控制器的处理（`ERR_PARAM_NUM`）
- 3.7.6–3.8.3 与 3.8.4 之外的状态帧：只做过 SDK 各 tag 静态比对，未上真机

## 已知风险

- **故障恢复全由操作者发起**，驱动绝不自动复位（[docs/fault_handling.md](docs/fault_handling.md)）
  —— impact: 无人值守时故障即停机
