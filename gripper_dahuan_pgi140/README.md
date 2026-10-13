# gripper_dahuan_pgi140

大寰（DH-Robotics）PGI-140-80 平行夹爪的描述包：xacro 宏 `gripper_dahuan_pgi140`
（`urdf/gripper_dahuan_pgi140_macro.xacro`）与网格（`collision/`）。

## 用法

```xml
<xacro:include filename="$(find gripper_dahuan_pgi140)/urdf/gripper_dahuan_pgi140_macro.xacro"/>
<xacro:gripper_dahuan_pgi140 tf_prefix="alice_" parent="tool0"/>
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `tf_prefix` | `""` | 所有连杆、关节的前缀，也用于 `parent` |
| `parent` | `tool0` | 安装连杆（不带前缀） |
| `xyz` / `rpy` | `0 0 0.0242` / `0 0 0` | 在 `parent` 中的安装位姿：底座网格最低点（z = −0.0242 m）落在法兰平面上；Fairino 的 `tool0` 上手指沿 `tool0` x 开合 |

- 根连杆 `<tf_prefix>gripper_base_link`，手指沿其 +z 伸出、沿 x 开合
- `<tf_prefix>left_finger_joint`：主动关节，0（闭合）– 0.04 m（全开），即单侧行程；对应控制器的开度 0–100 %
- `<tf_prefix>right_finger_joint`：mimic 左指，关节坐标系镜像，最大总开口 0.08 m
- 碰撞用不带线缆的底座网格（线缆是柔性的，当刚体会误报碰撞），可视用完整网格

在 Fairino 机械臂上随 ros2_control 使用：`fr_robot_driver` 的 `gripper:=pgi140`，见
[fr_robot_driver/README.md](../fr_robot_driver/README.md)「夹爪」一节。

## 安全

**激活必然张开到满行程**，不论手里夹着什么（[docs/sdk_quirks.md](../docs/sdk_quirks.md) G2）。
驱动从不自动激活；激活只由操作者通过 `~/gripper/activate` 服务发起，发起前确认手指之间无物、张开路径无障碍。
