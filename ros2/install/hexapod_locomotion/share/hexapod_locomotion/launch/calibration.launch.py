from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexapod_locomotion',
            executable='servo_driver',
            name='servo_driver',
            output='screen',
            parameters=[{
                'dry_run': True,
                'apply_offsets': False
            }]
        ),
        Node(
            package='hexapod_locomotion',
            executable='calibration',
            name='calibration',
            output='screen'
        ),
    ])
