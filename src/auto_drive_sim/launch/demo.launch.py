"""Launch the full lane-following demo."""

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge


def _start_gazebo(context):
    gui = LaunchConfiguration('gui').perform(context).lower() in (
        '1', 'true', 'yes')
    if LaunchConfiguration('lane_change_to').perform(context) and any(
            LaunchConfiguration(arg).perform(context).lower() in ('1', 'true', 'yes')
            for arg in ('lead_detector', 'lead_driver')):
        raise ValueError('The lane-change scenario requires an empty road')
    share = get_package_share_directory('auto_drive_sim')
    world_arg = LaunchConfiguration('world').perform(context)
    world = world_arg if os.path.isabs(world_arg) else \
        os.path.join(share, 'worlds', world_arg)
    models = os.path.join(share, 'models')

    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = models + (':' + existing if existing else '')

    if gui:
        command = ['gz', 'sim', '-r', world]
    else:
        command = ['gz', 'sim', '-s', '-r']
        if platform.system() != 'Darwin':
            command.append('--headless-rendering')
        command.append(world)

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        ExecuteProcess(cmd=command, output='screen'),
    ]


def generate_launch_description():
    """Build the demo launch description."""
    share = get_package_share_directory('auto_drive_sim')
    bridge_config = os.path.join(share, 'config', 'bridge.yaml')
    lead_bridge_config = os.path.join(share, 'config', 'lead_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start the Gazebo graphical client.',
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='true',
            description='Start the lane controller.',
        ),
        DeclareLaunchArgument(
            'foxglove',
            default_value='false',
            description='Start the Foxglove bridge on port 8765.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='car_world.sdf',
            description='World file name or absolute path.',
        ),
        DeclareLaunchArgument(
            'target_speed',
            default_value='1.5',
            description='Target speed in m/s.',
        ),
        DeclareLaunchArgument(
            'lane_strategy',
            default_value='single',
            description='Lane selection on the chosen world: single, rightmost, or leftmost.',
        ),
        DeclareLaunchArgument('lane_change_to', default_value='',
                              description='Optional one-time lane change on the empty two-lane test road.'),
        DeclareLaunchArgument(
            'lead_detector',
            default_value='false',
            description='Start the camera-based lead-vehicle detector.',
        ),
        DeclareLaunchArgument(
            'lead_driver',
            default_value='false',
            description='Start the moving lead-car driver and bridge.',
        ),
        DeclareLaunchArgument('lead_braking_scenario', default_value='false'),
        OpaqueFunction(function=_start_gazebo),
        RosGzBridge(
            bridge_name='car_bridge',
            config_file=bridge_config,
        ),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('lead_driver')),
            actions=[
                RosGzBridge(
                    bridge_name='lead_bridge',
                    config_file=lead_bridge_config,
                ),
                Node(
                    package='auto_drive_sim',
                    executable='lead_driver',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'start_x': 5.0,
                        'start_y': 13.19,
                        'braking_scenario': ParameterValue(
                            LaunchConfiguration('lead_braking_scenario'), value_type=bool),
                    }],
                ),
            ],
        ),
        Node(
            package='auto_drive_sim',
            executable='lane_perception',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'lane_strategy': LaunchConfiguration('lane_strategy'),
                'lane_change_to': ParameterValue(LaunchConfiguration('lane_change_to'), value_type=str),
            }],
        ),
        Node(
            package='auto_drive_sim',
            executable='lead_detector',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lead_detector')),
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('foxglove')),
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'use_sim_time': True,
            }],
        ),
        Node(
            package='auto_drive_sim',
            executable='lane_controller',
            output='screen',
            condition=IfCondition(LaunchConfiguration('controller')),
            parameters=[{
                'use_sim_time': True,
                'lead_enabled': ParameterValue(
                    LaunchConfiguration('lead_detector'), value_type=bool),
                'target_speed_mps': ParameterValue(
                    LaunchConfiguration('target_speed'), value_type=float),
            }],
        ),
    ])
