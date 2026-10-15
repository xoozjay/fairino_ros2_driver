import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    tf_prefix = LaunchConfiguration('tf_prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    gripper = LaunchConfiguration('gripper').perform(context)

    driver_share = get_package_share_directory('fr_robot_driver')
    moveit_share = get_package_share_directory('fr_moveit_config')

    urdf_file = os.path.join(driver_share, 'urdf', 'fr_controlled.urdf.xacro')
    srdf_file = os.path.join(moveit_share, 'srdf', 'fr.srdf.xacro')

    builder = MoveItConfigsBuilder('fr_robot', package_name='fr_moveit_config')
    builder.robot_description(
        file_path=urdf_file,
        mappings={
            'model': model,
            'tf_prefix': tf_prefix,
            'gripper': gripper,
            'use_mock_hardware': 'true',
        },
    )
    builder.robot_description_semantic(
        file_path=srdf_file,
        mappings={
            'model': model,
            'tf_prefix': tf_prefix,
            'gripper': gripper,
        },
    )
    builder.trajectory_execution(moveit_manage_controllers=False)
    builder.planning_pipelines(
        default_planning_pipeline='ompl',
        pipelines=['ompl', 'pilz_industrial_motion_planner'],
    )

    moveit_config = builder.to_moveit_configs()

    # Handle tf_prefix for kinematics and controller joint mappings
    arm_group = f'{tf_prefix}arm' if tf_prefix else 'arm'
    kin_dict = moveit_config.robot_description_kinematics.get('robot_description_kinematics', {})
    if 'arm' in kin_dict and arm_group != 'arm':
        kin_dict[arm_group] = kin_dict['arm']

    # Prefix joint names in trajectory controller if tf_prefix is present
    if tf_prefix:
        traj_exec = moveit_config.trajectory_execution
        if 'moveit_simple_controller_manager' in traj_exec:
            jtc = traj_exec.get('joint_trajectory_controller', {})
            if 'joints' in jtc:
                jtc['joints'] = [f'{tf_prefix}{j}' for j in jtc['joints']]

    # Load Pilz sequence capability
    moveit_config.move_group_capabilities['capabilities'] = (
        'pilz_industrial_motion_planner/MoveGroupSequenceAction '
        'pilz_industrial_motion_planner/MoveGroupSequenceService'
    )

    sub_launch = generate_move_group_launch(moveit_config)

    if namespace:
        return [
            GroupAction([
                PushRosNamespace(namespace),
                sub_launch,
            ])
        ]
    return [sub_launch]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='fairino5_v6',
                              description='Robot model in fr_description'),
        DeclareLaunchArgument('tf_prefix', default_value='',
                              description='TF/joint/link prefix'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS node and topic namespace'),
        DeclareLaunchArgument('gripper', default_value='none',
                              choices=['none', 'pgi140'],
                              description='Optional end-effector gripper'),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
