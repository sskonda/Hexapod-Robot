from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config_dir = Path(get_package_share_directory('hexapod_locomotion')) / 'config'

    return LaunchDescription([
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
                'dry_run': True,
                'apply_offsets': True
            }]
        ),
    ])
