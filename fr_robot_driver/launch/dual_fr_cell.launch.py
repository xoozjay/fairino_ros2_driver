# Copyright (c) 2026-present. fairino_ros2_driver contributors.
# Dual-robot workbench demo cell (UR tutorials' my_dual_robot_cell pattern):
# two Fairino robots on a 1.2 m x 1.8 m workbench, symmetric about the width
# centerline, each 0.4 m from the center (default model: fairino5_v6).
#
#   alice: x = -0.4, yaw 180 deg (faces +x, i.e. toward bob)
#   bob  : x = +0.4, yaw   0 deg (faces -x, i.e. toward alice)
#
# At the ZERO configuration both arms stretch across the table and end up
# exactly inside each other's volume (scripts/zero_pose_collision_check.py);
# the workbench initial pose folds each arm up toward its own table end.
#
# ros2_control keeps one controller_manager per robot (namespace + tf_prefix),
# so topics, controllers and TF frames never collide. A third, static
# robot_state_publisher shows the workbench itself in RViz.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'fr_robot_driver'


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory(PACKAGE)
    model = LaunchConfiguration('model').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context).lower() in ('true', '1')

    robot_args = {
        'model': model,
        'use_mock_hardware': use_mock_hardware,
        # scene-specific initial pose (NOT the standalone ready pose)
        'initial_positions_file': os.path.join(
            pkg_share, 'config', 'workbench_initial_positions.yaml'),
        'use_world_tf': 'true',
    }

    def robot(namespace, tf_prefix, xyz, quat):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'fr_control.launch.py')),
            launch_arguments={
                **robot_args,
                'namespace': namespace,
                'tf_prefix': tf_prefix,
                'world_origin_xyz': xyz,
                'world_origin_quat': quat,
            }.items(),
        )

    actions = [
        robot('alice', 'alice_', '-0.4 0 0.75', '0 0 1 0'),  # yaw 180 deg
        robot('bob', 'bob_', '0.4 0 0.75', '0 0 0 1'),       # yaw 0 deg

        # Static workbench description (world + table), its own rsp
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='table',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', os.path.join(
                        pkg_share, 'urdf', 'table_cell.urdf.xacro')]),
                    value_type=str),
            }],
        ),
    ]

    if launch_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'dual_cell.rviz')],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='fairino5_v6'),
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
