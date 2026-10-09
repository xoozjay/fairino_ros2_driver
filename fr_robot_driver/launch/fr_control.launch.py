# Copyright (c) 2026-present. fairino_ros2_driver contributors.
# Bringup for ONE Fairino robot under ros2_control (mock or real hardware),
# following the UR tutorials' my_robot_cell_control pattern.
#
# The same file is included multiple times by cell launch files (e.g.
# dual_fr_cell.launch.py) to build multi-robot environments: each robot gets
# its own namespace + tf_prefix, so topics, controllers and TF frames never
# collide.

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'fr_robot_driver'


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory(PACKAGE)

    model = LaunchConfiguration('model').perform(context)
    tf_prefix = LaunchConfiguration('tf_prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context)
    initial_positions_file = LaunchConfiguration('initial_positions_file').perform(context)
    use_world_tf = LaunchConfiguration('use_world_tf').perform(context).lower() in ('true', '1')
    world_origin_xyz = LaunchConfiguration('world_origin_xyz').perform(context)
    world_origin_quat = LaunchConfiguration('world_origin_quat').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context).lower() in ('true', '1')
    ip = LaunchConfiguration('ip').perform(context)
    cmd_t_ms = int(LaunchConfiguration('cmd_t_ms').perform(context))
    prefill = LaunchConfiguration('prefill').perform(context)
    allow_unverified = LaunchConfiguration('allow_unverified_version').perform(context)
    mock_gpio_commands = LaunchConfiguration('mock_gpio_commands').perform(context)
    activate_hardware = LaunchConfiguration('activate_hardware').perform(context).lower() in ('true', '1')
    mock = use_mock_hardware.lower() in ('true', '1')
    # Real hardware: one ServoJ segment of cmdT per controller cycle (docs/sdk_quirks.md S1).
    real_update_rate = 1000 // cmd_t_ms

    # Description: fr_description macro + ros2_control, in one document
    xacro_file = os.path.join(pkg_share, 'urdf', 'fr_controlled.urdf.xacro')
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' model:=', model,
        ' tf_prefix:=', tf_prefix,
        ' use_mock_hardware:=', use_mock_hardware,
        ' initial_positions_file:=', initial_positions_file,
        ' mock_gpio_commands:=', mock_gpio_commands,
        ' ip:=', ip,
        ' cmd_t_ms:=', str(cmd_t_ms),
        ' prefill:=', prefill,
        ' update_rate_hz:=', str(real_update_rate),
        ' namespace:=', namespace if namespace else "''",
        ' allow_unverified_version:=', allow_unverified,
    ])

    # Controllers: instantiate the template with this robot's tf_prefix and
    # namespace (root keys must be the namespaced controller names, ros2_control 4.x)
    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')
    ns_token = '/' + namespace if namespace else ''
    with open(controllers_file, 'r') as f:
        processed = (f.read()
                     .replace('__PREFIX__', tf_prefix)
                     .replace('__NS__', ns_token))
    controllers_dump = yaml.safe_load(processed)
    processed_file = os.path.join(
        tempfile.gettempdir(), f'fr_controllers_{tf_prefix.strip("_") or "robot"}.yaml')
    with open(processed_file, 'w') as f:
        f.write(processed)

    ns = namespace or None

    # controller_manager parameters: real hardware runs at 1000/cmdT Hz and enforces the URDF limits
    cm_params = dict(controllers_dump['controller_manager']['ros__parameters'])
    component = f'{tf_prefix}fr_robot'   # name of the <ros2_control> hardware component
    if not mock:
        cm_params['update_rate'] = real_update_rate
        cm_params['enforce_command_limits'] = True
        # Start configured but NOT active. If the controller manager activated the hardware itself
        # and a precondition failed (emergency stop, fault, not enabled), ros2_control would abort
        # the whole process, and with it the reset_error service. The activation below is an explicit
        # step whose failure leaves everything else running.
        cm_params['hardware_components_initial_state'] = {'inactive': [component]}

    # robot_state_publisher: URDF -> TF (frames carry tf_prefix already)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'publish_frequency': 50.0,
        }],
    )

    # controller_manager: one per robot (namespaced -> no cross-talk)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            cm_params,
        ],
    )

    # Declarative controllers, spawned after the controller_manager
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster', '--param-file', processed_file],
    )
    jtc = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_trajectory_controller', '--param-file', processed_file],
    )

    nodes = [rsp, control_node, jsb]
    if mock or not activate_hardware:
        nodes.append(jtc)
    else:
        cm_name = '/' + (namespace + '/' if namespace else '') + 'controller_manager'
        activate = ExecuteProcess(
            cmd=['ros2', 'control', 'set_hardware_component_state', component, 'active',
                 '--controller-manager', cm_name],
            output='screen')
        # the trajectory controller needs the command interfaces, which exist once active
        nodes.append(activate)
        nodes.append(RegisterEventHandler(OnProcessExit(target_action=activate, on_exit=[jtc])))

    # Optional: attach the robot base to a shared world frame
    if use_world_tf:
        world_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=namespace,
            arguments=[
                '--x', world_origin_xyz.split(' ')[0],
                '--y', world_origin_xyz.split(' ')[1],
                '--z', world_origin_xyz.split(' ')[2],
                '--qx', world_origin_quat.split(' ')[0],
                '--qy', world_origin_quat.split(' ')[1],
                '--qz', world_origin_quat.split(' ')[2],
                '--qw', world_origin_quat.split(' ')[3],
                '--frame-id', 'world',
                '--child-frame-id', tf_prefix + 'base_link',
            ],
        )
        nodes.append(world_tf)

    if launch_rviz:
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('fr_description'), 'rviz', 'view_robot.rviz')],
        )
        nodes.append(rviz)

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='fairino20_v6'),
        DeclareLaunchArgument('tf_prefix', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('initial_positions_file',
                              default_value=os.path.join(pkg_share, 'config', 'initial_positions.yaml')),
        DeclareLaunchArgument('use_world_tf', default_value='false'),
        DeclareLaunchArgument('world_origin_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('world_origin_quat', default_value='0 0 0 1'),  # quaternion
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('ip', default_value='192.168.58.2',
                              description='controller address (real hardware only)'),
        DeclareLaunchArgument('cmd_t_ms', default_value='8',
                              description='ServoJ cmdT in whole ms; the controller manager runs at 1000/cmd_t_ms Hz'),
        DeclareLaunchArgument('prefill', default_value='3',
                              description='standing ServoJ segments at activation (jitter buffer)'),
        DeclareLaunchArgument('allow_unverified_version', default_value='false'),
        DeclareLaunchArgument('activate_hardware', default_value='true',
                              description='real hardware: activate the component after startup '
                                          '(false: leave it inactive, e.g. to reset a fault first)'),
        DeclareLaunchArgument('mock_gpio_commands', default_value='false',
                              description='mock only: loop-back command interfaces on fr_status'),
        OpaqueFunction(function=launch_setup),
    ])
