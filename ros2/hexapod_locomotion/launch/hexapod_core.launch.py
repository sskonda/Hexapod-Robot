from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
            output='screen'
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
