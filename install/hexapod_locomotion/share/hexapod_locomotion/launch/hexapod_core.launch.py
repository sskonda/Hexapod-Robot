from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config_dir = Path(get_package_share_directory('hexapod_locomotion')) / 'config'
    servo_dry_run = LaunchConfiguration('servo_dry_run')
    apply_offsets = LaunchConfiguration('apply_offsets')

    return LaunchDescription([
        DeclareLaunchArgument(
            'servo_dry_run',
            default_value='true',
            description='When true, servo_driver logs commands instead of driving hardware.',
        ),
        DeclareLaunchArgument(
            'apply_offsets',
            default_value='true',
            description='Apply saved calibration offsets in servo_driver.',
        ),
        Node(
            package='hexapod_locomotion',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen'
        ),
        Node(
            package='hexapod_locomotion',
            executable='locomotion',
            name='locomotion',
            output='screen',
            parameters=[str(config_dir / 'locomotion.yaml')]
        ),
        Node(
            package='hexapod_locomotion',
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            parameters=[{
                'dry_run': servo_dry_run,
                'apply_offsets': apply_offsets
            }]
        ),
    ])
