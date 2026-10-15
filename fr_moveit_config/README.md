# fr_moveit_config

MoveIt 2 configuration and acceptance testing for Fairino robots.

## Features

- **Parameterized configuration**: Single handwritten configuration package supporting all Fairino models (`fairino3_v6`, `fairino3_mt_v6`, `fairino5_v6`, `fairino10_v6`, `fairino16_v6`, `fairino20_v6`, `fairino30_v6`, `fr3wml`, `fr3wms`, `fr5wm`), arbitrary `tf_prefix`, and optional end-effector grippers (DH-Robotics `pgi140`).
- **Planners**: Default OMPL planning pipeline with Pilz Industrial Motion Planner (`PTP`, `LIN`, `CIRC`, and sequence execution capability `sequence_move_group`).
- **Safety**:
  - Finger-to-finger CAD self-collision is disabled per design.
  - Arm-to-gripper collisions are strictly preserved to catch dangerous configurations.
  - Controller execution duration margin configured to 31.0 s for the gripper controller.

## Quick Start

### 1. Launch Robot Control (Mock or Real)

```bash
# Mock robot
ros2 launch fr_robot_driver fr_control.launch.py model:=fairino5_v6 use_mock_hardware:=true

# Or real robot
ros2 launch fr_robot_driver fr_control.launch.py model:=fairino5_v6 ip:=192.168.58.2
```

### 2. Launch MoveIt Move Group

```bash
ros2 launch fr_moveit_config move_group.launch.py model:=fairino5_v6
```

Optional arguments:
- `model`: robot model (default `fairino5_v6`)
- `tf_prefix`: link/joint prefix (default `""`)
- `namespace`: ROS namespace (default `""`)
- `gripper`: end-effector gripper (`none` or `pgi140`)

### 3. Launch RViz (Optional)

```bash
ros2 launch fr_moveit_config moveit_rviz.launch.py model:=fairino5_v6
```

## Reproducing Collision Matrices

Self-collision disabling matrices for each robot model under `config/<model>/collisions.srdf.xacro` are generated using MoveIt Setup Assistant's `collisions_updater`:

```bash
# Example for fairino5_v6:
xacro $(ros2 pkg prefix --share fr_robot_driver)/urdf/fr_controlled.urdf.xacro model:=fairino5_v6 > /tmp/robot.urdf
echo '<robot name="fr_robot"/>' > /tmp/dummy.srdf
/opt/ros/jazzy/lib/moveit_setup_assistant/collisions_updater \
  --urdf /tmp/robot.urdf \
  --srdf /tmp/dummy.srdf \
  --output /tmp/out.srdf \
  --default --always --trials 10000
```
