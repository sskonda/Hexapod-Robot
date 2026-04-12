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
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
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
    explorer_footprint_radius_m = LaunchConfiguration('explorer_footprint_radius_m')
    explorer_wall_clearance_margin_m = LaunchConfiguration('explorer_wall_clearance_margin_m')
    explorer_clearance_window_deg = LaunchConfiguration('explorer_clearance_window_deg')
    explorer_min_gap_width_deg = LaunchConfiguration('explorer_min_gap_width_deg')
    explorer_reverse_avoidance_deg = LaunchConfiguration('explorer_reverse_avoidance_deg')
    explorer_max_replan_attempts = LaunchConfiguration('explorer_max_replan_attempts')
    explorer_forward_bias_weight = LaunchConfiguration('explorer_forward_bias_weight')
    explorer_min_progress_m = LaunchConfiguration('explorer_min_progress_m')
    explorer_recovery_backup_m = LaunchConfiguration('explorer_recovery_backup_m')
    crab_follower_speed_mps = LaunchConfiguration('crab_follower_speed_mps')
    crab_follower_goal_tolerance_m = LaunchConfiguration('crab_follower_goal_tolerance_m')

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
            'laser_frame',
            default_value='laser',
            description='TF frame of the LiDAR sensor — must match frame_id in /scan messages.',
        ),
        DeclareLaunchArgument(
            'laser_x',
            default_value='0.0',
            description='X offset of LiDAR from base_link in metres (forward +).',
        ),
        DeclareLaunchArgument(
            'laser_y',
            default_value='0.0',
            description='Y offset of LiDAR from base_link in metres (left +).',
        ),
        DeclareLaunchArgument(
            'laser_z',
            default_value='0.0',
            description='Z offset of LiDAR from base_link in metres (up +).',
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
        # --- Geometry reference (4 ft × 4 ft tiles, 1.5 ft robot radius) ------
        # Tile full width : 4 ft  = 1.2192 m
        # Half-tile       : 2 ft  = 0.6096 m  (centre to wall)
        # Robot radius    : 1.5 ft = 0.4572 m
        # Body-to-wall clearance when centred: 0.6096 - 0.4572 = 0.152 m (6 in)
        # Min traversable gap: 2 × (footprint + margin) = 2 × 0.5572 = 1.114 m
        #   → 4 ft corridor = 1.2192 m  ✓  (~8 cm of angular margin each side)
        # stop_distance = 0.72 m → robot edge clears wall by 0.72 - 0.4572 = 0.26 m
        # -----------------------------------------------------------------------
        DeclareLaunchArgument(
            'explorer_stop_distance_m',
            default_value='0.72',
            description=(
                'LiDAR clearance below which the robot stops and replans. '
                'Set to half-tile (0.6096 m) + 0.11 m safety margin for a '
                '1.5 ft radius robot in 4 ft corridors.'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_open_distance_m',
            default_value='0.90',
            description=(
                'Preferred clearance for scoring open gaps. '
                'Must exceed stop_distance; 0.90 m ≈ 75 % of a 4 ft tile.'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_goal_backoff_m',
            default_value='0.60',
            description=(
                'Distance the rolling goal is kept back from the detected obstacle. '
                'Set above footprint + margin (0.5572 m) so the goal never '
                'falls inside the wall.'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_max_goal_distance_m',
            default_value='1.50',
            description='Maximum rolling-path look-ahead (m). One full 4 ft tile = 1.2192 m.',
        ),
        DeclareLaunchArgument(
            'explorer_min_goal_distance_m',
            default_value='0.25',
            description='Minimum useful move when space is very tight.',
        ),
        DeclareLaunchArgument(
            'explorer_footprint_radius_m',
            default_value='0.4572',
            description='Robot circular footprint radius: 1.5 ft = 0.4572 m.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_clearance_margin_m',
            default_value='0.10',
            description=(
                'Safety buffer added beyond footprint_radius when checking '
                'minimum wall clearance and gap width.'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_clearance_window_deg',
            default_value='20.0',
            description=(
                'Half-width of the LiDAR sector used to evaluate a candidate '
                'heading. Wider (20 °) suits the larger 4 ft tile geometry.'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_min_gap_width_deg',
            default_value='18.0',
            description='Minimum angular gap width to be considered traversable.',
        ),
        DeclareLaunchArgument(
            'explorer_reverse_avoidance_deg',
            default_value='70.0',
            description='Penalty sector that discourages selecting the direction just travelled.',
        ),
        DeclareLaunchArgument(
            'explorer_max_replan_attempts',
            default_value='8',
            description='Consecutive low-travel stops before triggering a recovery backup.',
        ),
        DeclareLaunchArgument(
            'explorer_min_progress_m',
            default_value='0.15',
            description=(
                'Minimum travel (m) to count a stop as progress. '
                'Raised to 0.15 m for 4 ft tiles (was 0.10 m).'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_recovery_backup_m',
            default_value='0.30',
            description=(
                'Backup distance when stuck recovery fires. '
                'Raised to 0.30 m for 4 ft tiles (was 0.20 m).'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_forward_bias_weight',
            default_value='1.5',
            description='Bias toward selecting the forward heading on startup; 0.0 disables.',
        ),
        DeclareLaunchArgument(
            'crab_follower_speed_mps',
            default_value='0.04',
            description='Constant travel speed for the crab path follower in m/s.',
        ),
        DeclareLaunchArgument(
            'crab_follower_goal_tolerance_m',
            default_value='0.08',
            description='Distance from the rolling goal at which the follower considers itself arrived.',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '--x', laser_x, '--y', laser_y, '--z', laser_z,
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', base_frame, '--child-frame-id', laser_frame,
            ],
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
                'footprint_radius_m': explorer_footprint_radius_m,
                'wall_clearance_margin_m': explorer_wall_clearance_margin_m,
                'clearance_window_deg': explorer_clearance_window_deg,
                'min_gap_width_deg': explorer_min_gap_width_deg,
                'reverse_avoidance_deg': explorer_reverse_avoidance_deg,
                'max_replan_attempts': explorer_max_replan_attempts,
                'forward_bias_weight': explorer_forward_bias_weight,
                'min_progress_m': explorer_min_progress_m,
                'recovery_backup_m': explorer_recovery_backup_m,
            }],
        ),
        Node(
            package='hexapod_locomotion',
            executable='crab_path_follower',
            name='crab_path_follower',
            output='screen',
            condition=IfCondition(enable_explorer),
            parameters=[{
                'path_topic':          path_topic,
                'odom_topic':          odom_topic,
                'cmd_vel_topic':       'cmd_vel',
                'constant_speed_mps':  crab_follower_speed_mps,
                'goal_tolerance_m':    crab_follower_goal_tolerance_m,
                'path_timeout_sec':    1.0,
                'cmd_vel_rate_hz':     20.0,
            }],
        ),
    ])
