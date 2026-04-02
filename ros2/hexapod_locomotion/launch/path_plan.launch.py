from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory('hexapod_locomotion'))
    core_launch = share_dir / 'launch' / 'hexapod_core.launch.py'

    servo_dry_run = LaunchConfiguration('servo_dry_run')
    apply_offsets = LaunchConfiguration('apply_offsets')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    startup_delay_sec = LaunchConfiguration('startup_delay_sec')
    forward_distance_m = LaunchConfiguration('forward_distance_m')
    backward_distance_m = LaunchConfiguration('backward_distance_m')
    linear_speed_mps = LaunchConfiguration('linear_speed_mps')

    return LaunchDescription([
        DeclareLaunchArgument(
            'servo_dry_run',
            default_value='false',
            description='When true, servo_driver logs commands instead of driving hardware.',
        ),
        DeclareLaunchArgument(
            'apply_offsets',
            default_value='true',
            description='Apply saved calibration offsets in servo_driver.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='20.0',
            description='How often the path planner publishes cmd_vel commands.',
        ),
        DeclareLaunchArgument(
            'startup_delay_sec',
            default_value='1.0',
            description='Delay before the predefined path starts.',
        ),
        DeclareLaunchArgument(
            'forward_distance_m',
            default_value='0.5',
            description='Forward distance for the predefined path in meters.',
        ),
        DeclareLaunchArgument(
            'backward_distance_m',
            default_value='0.5',
            description='Backward distance for the predefined path in meters.',
        ),
        DeclareLaunchArgument(
            'linear_speed_mps',
            default_value='0.05',
            description='Linear speed for the predefined path in meters per second.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core_launch)),
            launch_arguments={
                'servo_dry_run': servo_dry_run,
                'apply_offsets': apply_offsets,
            }.items(),
        ),
        Node(
            package='hexapod_locomotion',
            executable='path_plan',
            name='path_plan',
            output='screen',
            parameters=[{
                'publish_rate_hz': publish_rate_hz,
                'startup_delay_sec': startup_delay_sec,
                'forward_distance_m': forward_distance_m,
                'backward_distance_m': backward_distance_m,
                'linear_speed_mps': linear_speed_mps,
            }],
        ),
    ])
