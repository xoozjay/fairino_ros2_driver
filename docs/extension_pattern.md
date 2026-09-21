# 扩展方式：宏 + 参数文件（ur_description 模式）

`fr_description` 采用参数化宏架构，所有扩展在 `src/` 内完成，
`references/fairino_vendor/` 只读。

## 新增型号

1. 在 `config/<model>/` 准备 4 个参数 YAML（格式参考现有型号；
   若有该型号的 vendor URDF，可参考 `scripts/generate_from_vendor.py` 的提取逻辑）：
   - `default_kinematics.yaml` — 关节原点/轴 + `flange_offset`（以 `docs/robot_dh_params.md` 的 Joint6 d 为准）
   - `joint_limits.yaml` — 关节限位
   - `physical_parameters.yaml` — 质量/质心/惯量
   - `visual_parameters.yaml` — 可视/碰撞 mesh（`package://fr_description/meshes/<model>/...`）
2. mesh 放入 `meshes/<model>/`；
3. **宏无需改动**（所有型号共享同一拓扑：6 关节 + flange + tool0）；
4. 如需快捷可视化，把型号加进 `launch/view_robot.launch.py` 的 `MODELS` 列表；
5. 按下文验证。

## 组装到更大的单元（双臂/工作单元）

include 宏并多次实例化，`tf_prefix` 区分（这正是替换 vendor URDF 的动机）：

```xml
<xacro:include filename="$(find fr_description)/urdf/fr_macro.xacro"/>
<link name="world"/>
<xacro:fr_robot parent="world" tf_prefix="alice_" .../>
<xacro:fr_robot parent="world" origin_xyz="1 0 0" tf_prefix="bob_" .../>
```

## 修改后必须验证

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash   # 工作区根目录

# 1) 单个型号（check_urdf 需要 文件参数，不支持管道）
xacro src/fairino_ros2_driver/fr_description/urdf/fr.urdf.xacro \
  model:=<model> > /tmp/check.urdf && check_urdf /tmp/check.urdf

# 2) 多机组装：宏实例化 ≥2 次的 xacro，同样 xacro + check_urdf，
#    额外要求：无重复 link/joint 名
```

要求：xacro 无报错、check_urdf 解析成功、frame 链末端为 `flange -> tool0`。

## 命名约定（新包已规范化）

| 类别 | 约定 |
|---|---|
| link | `base_link … wrist_3_link` + `flange` + `tool0`（snake_case，对齐 ur_description） |
| joint | `shoulder_pan_joint … wrist_3_joint`（语义化，替代 vendor 的 j1..j6/joint1..joint6） |
| 型号目录 | 小写：`fairino20_v6`、`fr3wml` |
| 多机 | `tf_prefix`，如 `alice_` |
