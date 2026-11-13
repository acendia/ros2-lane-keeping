"""Launch the car world, the ROS-Gazebo bridge, and optional teleoperation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


def _start_gazebo(context):
    gui = LaunchConfiguration('gui').perform(context).lower() in (
        '1', 'true', 'yes')
    share = get_package_share_directory('auto_drive_sim')
    world = os.path.join(share, 'worlds', 'car_world.sdf')
    models = os.path.join(share, 'models')

    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = models + (':' + existing if existing else '')

    if gui:
        command = ['gz', 'sim', '-r', world]
    else:
        command = ['gz', 'sim', '-s', '-r', '--headless-rendering', world]

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        ExecuteProcess(cmd=command, output='screen'),
    ]


def generate_launch_description():
    """Build the manual-drive launch description."""
    share = get_package_share_directory('auto_drive_sim')
    bridge_config = os.path.join(share, 'config', 'bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start the Gazebo graphical client.',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Start the keyboard teleoperation node.',
        ),
        OpaqueFunction(function=_start_gazebo),
        RosGzBridge(
            bridge_name='car_bridge',
            config_file=bridge_config,
        ),
        Node(
            package='auto_drive_sim',
            executable='teleop',
            output='screen',
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
    ])
