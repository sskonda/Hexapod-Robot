from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    enable_explorer = LaunchConfiguration('enable_explorer')
    odom_topic = LaunchConfiguration('odom_topic')
    path_topic = LaunchConfiguration('path_topic')
    stop_point_topic = LaunchConfiguration('stop_point_topic')
    explorer_control_rate_hz = LaunchConfiguration('explorer_control_rate_hz')
    explorer_path_publish_period_sec = LaunchConfiguration('explorer_path_publish_period_sec')
    explorer_decision_pause_sec = LaunchConfiguration('explorer_decision_pause_sec')
    explorer_stop_distance_m = LaunchConfiguration('explorer_stop_distance_m')
    explorer_open_distance_m = LaunchConfiguration('explorer_open_distance_m')
    explorer_goal_backoff_m = LaunchConfiguration('explorer_goal_backoff_m')
    explorer_max_goal_distance_m = LaunchConfiguration('explorer_max_goal_distance_m')
    explorer_min_goal_distance_m = LaunchConfiguration('explorer_min_goal_distance_m')
    explorer_clearance_window_deg = LaunchConfiguration('explorer_clearance_window_deg')
    explorer_min_gap_width_deg = LaunchConfiguration('explorer_min_gap_width_deg')
    explorer_reverse_avoidance_deg = LaunchConfiguration('explorer_reverse_avoidance_deg')

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
        DeclareLaunchArgument(
            'enable_explorer',
            default_value='true',
            description='Launch the lidar gap-following exploration node.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odom',
            description='Odometry topic consumed by the exploration node.',
        ),
        DeclareLaunchArgument(
            'path_topic',
            default_value='path',
            description='Path topic published by the exploration node.',
        ),
        DeclareLaunchArgument(
            'stop_point_topic',
            default_value='decision_point',
            description='PointStamped topic published when the robot stops to choose a new direction.',
        ),
        DeclareLaunchArgument(
            'explorer_control_rate_hz',
            default_value='5.0',
            description='Explorer control loop rate in Hz.',
        ),
        DeclareLaunchArgument(
            'explorer_path_publish_period_sec',
            default_value='0.5',
            description='How often the rolling path is refreshed while the robot is moving.',
        ),
        DeclareLaunchArgument(
            'explorer_decision_pause_sec',
            default_value='0.75',
            description='Pause after reaching a wall before picking a new heading.',
        ),
        DeclareLaunchArgument(
            'explorer_stop_distance_m',
            default_value='0.55',
            description='Minimum allowed lidar clearance before stopping and replanning.',
        ),
        DeclareLaunchArgument(
            'explorer_open_distance_m',
            default_value='0.90',
            description='Preferred clearance used to score open gaps.',
        ),
        DeclareLaunchArgument(
            'explorer_goal_backoff_m',
            default_value='0.35',
            description='Distance kept between the rolling goal point and the detected obstacle.',
        ),
        DeclareLaunchArgument(
            'explorer_max_goal_distance_m',
            default_value='1.25',
            description='Maximum distance for the rolling path target.',
        ),
        DeclareLaunchArgument(
            'explorer_min_goal_distance_m',
            default_value='0.20',
            description='Minimum distance for the rolling path target when space is tight.',
        ),
        DeclareLaunchArgument(
            'explorer_clearance_window_deg',
            default_value='12.0',
            description='Half-width of the lidar sector used to evaluate a candidate heading.',
        ),
        DeclareLaunchArgument(
            'explorer_min_gap_width_deg',
            default_value='18.0',
            description='Minimum angular width required for a gap to be considered traversable.',
        ),
        DeclareLaunchArgument(
            'explorer_reverse_avoidance_deg',
            default_value='70.0',
            description='How aggressively the explorer avoids selecting the direction it just came from.',
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
        Node(
            package='hexapod_slam',
            executable='gap_following_explorer',
            name='gap_following_explorer',
            output='screen',
            condition=IfCondition(enable_explorer),
            parameters=[{
                'scan_topic': scan_topic,
                'odom_topic': odom_topic,
                'path_topic': path_topic,
                'stop_point_topic': stop_point_topic,
                'control_rate_hz': explorer_control_rate_hz,
                'path_publish_period_sec': explorer_path_publish_period_sec,
                'decision_pause_sec': explorer_decision_pause_sec,
                'stop_distance_m': explorer_stop_distance_m,
                'open_distance_m': explorer_open_distance_m,
                'goal_backoff_m': explorer_goal_backoff_m,
                'max_goal_distance_m': explorer_max_goal_distance_m,
                'min_goal_distance_m': explorer_min_goal_distance_m,
                'clearance_window_deg': explorer_clearance_window_deg,
                'min_gap_width_deg': explorer_min_gap_width_deg,
                'reverse_avoidance_deg': explorer_reverse_avoidance_deg,
            }],
        ),
    ])
