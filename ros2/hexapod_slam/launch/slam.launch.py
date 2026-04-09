from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = Path(get_package_share_directory('hexapod_slam'))
    default_params = share_dir / 'config' / 'slam_toolbox.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock if available.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan',
            description='LaserScan topic consumed by SLAM.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(default_params),
            description='Full path to the slam_toolbox parameter file.',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='TF frame published by SLAM for the map.',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='TF frame used for local odometry.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base TF frame.',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'map_frame': map_frame,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                },
            ],
            remappings=[
                ('scan', scan_topic),
            ],
        ),
    ])
