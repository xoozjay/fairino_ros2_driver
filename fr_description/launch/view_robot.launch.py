# Copyright (c) 2025 fairino_ros2_driver contributors
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

"""Visualize any supported Fairino model in RViz with a joint slider GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MODELS = [
    "fairino3_v6",
    "fairino3_mt_v6",
    "fairino5_v6",
    "fairino10_v6",
    "fairino16_v6",
    "fairino20_v6",
    "fairino30_v6",
    "fr3wml",
    "fr3wms",
    "fr5wm",
]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "model",
            default_value="fairino20_v6",
            choices=MODELS,
            description="Fairino robot model to visualize.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for all frames (useful for multi-robot setups).",
        ),
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("fr_description"), "urdf", "fr.urdf.xacro"]
            ),
            " ",
            "model:=",
            LaunchConfiguration("model"),
            " ",
            "tf_prefix:=",
            LaunchConfiguration("tf_prefix"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare("fr_description"), "rviz", "view_robot.rviz"]
        )],
    )

    return LaunchDescription(
        declared_arguments
        + [robot_state_publisher_node, joint_state_publisher_gui_node, rviz_node]
    )
