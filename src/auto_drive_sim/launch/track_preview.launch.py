"""Launch the generated oval world in Gazebo Harmonic."""

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_gazebo(context):
    gui = LaunchConfiguration('gui').perform(context).lower() in (
        '1', 'true', 'yes')
    share = get_package_share_directory('auto_drive_sim')
    world = os.path.join(share, 'worlds', 'lane_world.sdf')
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
    """Build the preview launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start the Gazebo graphical client.',
        ),
        OpaqueFunction(function=_start_gazebo),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
    ])
