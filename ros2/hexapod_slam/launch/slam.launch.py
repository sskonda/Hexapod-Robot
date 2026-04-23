from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share_dir = Path(get_package_share_directory('hexapod_slam'))
    default_slam_params = share_dir / 'config' / 'slam_toolbox.yaml'
    default_ekf_params = share_dir / 'config' / 'ekf.yaml'

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params_file = LaunchConfiguration('slam_params_file')
    robot_localization_params_file = LaunchConfiguration('robot_localization_params_file')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    enable_explorer = LaunchConfiguration('enable_explorer')
    enable_robot_localization = LaunchConfiguration('enable_robot_localization')
    autostart_slam = LaunchConfiguration('autostart_slam')
    laser_frame = LaunchConfiguration('laser_frame')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    odom_topic = LaunchConfiguration('odom_topic')
    raw_odom_topic = LaunchConfiguration('raw_odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    path_topic = LaunchConfiguration('path_topic')
    stop_point_topic = LaunchConfiguration('stop_point_topic')
    raw_cmd_vel_topic = LaunchConfiguration('raw_cmd_vel_topic')
    safe_cmd_vel_topic = LaunchConfiguration('safe_cmd_vel_topic')
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
    explorer_corner_mode_enabled = LaunchConfiguration('explorer_corner_mode_enabled')
    explorer_corner_forward_blocked_distance_m = LaunchConfiguration(
        'explorer_corner_forward_blocked_distance_m'
    )
    explorer_corner_side_open_distance_m = LaunchConfiguration(
        'explorer_corner_side_open_distance_m'
    )
    explorer_corner_clearance_window_deg = LaunchConfiguration(
        'explorer_corner_clearance_window_deg'
    )
    explorer_corner_min_turn_angle_deg = LaunchConfiguration(
        'explorer_corner_min_turn_angle_deg'
    )
    explorer_corner_max_turn_angle_deg = LaunchConfiguration(
        'explorer_corner_max_turn_angle_deg'
    )
    explorer_corner_preferred_turn_angle_deg = LaunchConfiguration(
        'explorer_corner_preferred_turn_angle_deg'
    )
    explorer_corner_goal_distance_m = LaunchConfiguration('explorer_corner_goal_distance_m')
    explorer_corner_outer_wall_bias_m = LaunchConfiguration(
        'explorer_corner_outer_wall_bias_m'
    )
    explorer_corner_commit_time_sec = LaunchConfiguration('explorer_corner_commit_time_sec')
    explorer_wall_follow_enabled = LaunchConfiguration('explorer_wall_follow_enabled')
    explorer_wall_follow_trigger_replans = LaunchConfiguration(
        'explorer_wall_follow_trigger_replans'
    )
    explorer_wall_follow_front_blocked_distance_m = LaunchConfiguration(
        'explorer_wall_follow_front_blocked_distance_m'
    )
    explorer_wall_follow_wall_detect_distance_m = LaunchConfiguration(
        'explorer_wall_follow_wall_detect_distance_m'
    )
    explorer_wall_follow_target_standoff_m = LaunchConfiguration(
        'explorer_wall_follow_target_standoff_m'
    )
    explorer_wall_follow_search_min_angle_deg = LaunchConfiguration(
        'explorer_wall_follow_search_min_angle_deg'
    )
    explorer_wall_follow_search_max_angle_deg = LaunchConfiguration(
        'explorer_wall_follow_search_max_angle_deg'
    )
    explorer_wall_follow_wall_window_deg = LaunchConfiguration(
        'explorer_wall_follow_wall_window_deg'
    )
    explorer_wall_follow_tangent_window_deg = LaunchConfiguration(
        'explorer_wall_follow_tangent_window_deg'
    )
    explorer_wall_follow_goal_distance_m = LaunchConfiguration(
        'explorer_wall_follow_goal_distance_m'
    )
    explorer_wall_follow_midpoint_distance_m = LaunchConfiguration(
        'explorer_wall_follow_midpoint_distance_m'
    )
    explorer_wall_follow_goal_backoff_m = LaunchConfiguration(
        'explorer_wall_follow_goal_backoff_m'
    )
    explorer_wall_follow_standoff_gain = LaunchConfiguration(
        'explorer_wall_follow_standoff_gain'
    )
    explorer_wall_follow_forward_bias = LaunchConfiguration(
        'explorer_wall_follow_forward_bias'
    )
    explorer_wall_follow_min_forward_component = LaunchConfiguration(
        'explorer_wall_follow_min_forward_component'
    )
    explorer_wall_follow_max_correction_ratio = LaunchConfiguration(
        'explorer_wall_follow_max_correction_ratio'
    )
    explorer_wall_follow_min_tangent_clearance_m = LaunchConfiguration(
        'explorer_wall_follow_min_tangent_clearance_m'
    )
    explorer_wall_follow_max_follow_angle_deg = LaunchConfiguration(
        'explorer_wall_follow_max_follow_angle_deg'
    )
    explorer_wall_follow_commit_time_sec = LaunchConfiguration(
        'explorer_wall_follow_commit_time_sec'
    )
    explorer_wall_follow_cooldown_time_sec = LaunchConfiguration(
        'explorer_wall_follow_cooldown_time_sec'
    )
    explorer_wall_follow_exit_forward_clearance_m = LaunchConfiguration(
        'explorer_wall_follow_exit_forward_clearance_m'
    )
    explorer_wall_follow_exit_heading_max_deg = LaunchConfiguration(
        'explorer_wall_follow_exit_heading_max_deg'
    )
    explorer_wall_follow_stuck_time_sec = LaunchConfiguration(
        'explorer_wall_follow_stuck_time_sec'
    )
    explorer_max_replan_attempts = LaunchConfiguration('explorer_max_replan_attempts')
    explorer_forward_bias_weight = LaunchConfiguration('explorer_forward_bias_weight')
    explorer_max_yaw_drift_deg = LaunchConfiguration('explorer_max_yaw_drift_deg')
    explorer_min_progress_m = LaunchConfiguration('explorer_min_progress_m')
    explorer_recovery_backup_m = LaunchConfiguration('explorer_recovery_backup_m')
    crab_follower_speed_mps = LaunchConfiguration('crab_follower_speed_mps')
    crab_follower_goal_tolerance_m = LaunchConfiguration('crab_follower_goal_tolerance_m')
    crab_follower_yaw_correction_gain = LaunchConfiguration('crab_follower_yaw_correction_gain')
    crab_follower_max_angular_speed_rad_s = LaunchConfiguration(
        'crab_follower_max_angular_speed_rad_s'
    )
    crab_follower_yaw_deadband_deg = LaunchConfiguration('crab_follower_yaw_deadband_deg')
    cmd_vel_yaw_offset_rad = LaunchConfiguration('cmd_vel_yaw_offset_rad')
    safety_stop_distance_m = LaunchConfiguration('safety_stop_distance_m')
    safety_slowdown_distance_m = LaunchConfiguration('safety_slowdown_distance_m')
    safety_clearance_window_deg = LaunchConfiguration('safety_clearance_window_deg')
    safety_side_stop_distance_m = LaunchConfiguration('safety_side_stop_distance_m')
    safety_side_slowdown_distance_m = LaunchConfiguration('safety_side_slowdown_distance_m')
    safety_side_clearance_window_deg = LaunchConfiguration('safety_side_clearance_window_deg')
    safety_max_side_push_ratio = LaunchConfiguration('safety_max_side_push_ratio')

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
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
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock if available.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan',
            description='LaserScan topic consumed by SLAM and the safety filter.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(default_slam_params),
            description='Full path to the slam_toolbox parameter file.',
        ),
        DeclareLaunchArgument(
            'robot_localization_params_file',
            default_value=str(default_ekf_params),
            description='Full path to the robot_localization EKF parameter file.',
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
            description='Launch the LiDAR gap-following explorer, follower, and safety filter.',
        ),
        DeclareLaunchArgument(
            'enable_robot_localization',
            default_value='false',
            description=(
                'Launch robot_localization to fuse raw odom and IMU into the odom topic '
                'used by the explorer, follower, and SLAM.'
            ),
        ),
        DeclareLaunchArgument(
            'autostart_slam',
            default_value='true',
            description=(
                'Automatically configure and activate the slam_toolbox lifecycle node. '
                'Disable for manual lifecycle debugging.'
            ),
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
            'laser_roll',
            default_value='0.0',
            description='Roll offset of LiDAR from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'laser_pitch',
            default_value='0.0',
            description='Pitch offset of LiDAR from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'laser_yaw',
            default_value='0.0',
            description='Yaw offset of LiDAR from base_link in radians.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='odom',
            description='Odometry topic consumed by the explorer and follower.',
        ),
        DeclareLaunchArgument(
            'raw_odom_topic',
            default_value='odom/raw',
            description='Raw locomotion odometry topic fed into robot_localization when enabled.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='IMU topic fused by robot_localization when enabled.',
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
            'raw_cmd_vel_topic',
            default_value='cmd_vel_nav',
            description='Intermediate planner/follower cmd_vel topic before the safety filter.',
        ),
        DeclareLaunchArgument(
            'safe_cmd_vel_topic',
            default_value='cmd_vel',
            description='Command topic after the scan-based safety filter.',
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
        #   -> 4 ft corridor = 1.2192 m  ✓  (~8 cm of angular margin each side)
        # stop_distance = 0.60 m -> robot edge clears wall by 0.60 - 0.4572 = 0.14 m
        # -----------------------------------------------------------------------
        DeclareLaunchArgument(
            'explorer_stop_distance_m',
            default_value='0.60',
            description=(
                'LiDAR clearance below which the robot stops and replans. '
                'Tune this to the robot footprint and maze width; 0.60 m is a '
                'safer starting point for the current hexapod hardware.'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_open_distance_m',
            default_value='0.72',
            description=(
                'Preferred clearance for scoring open gaps. '
                'Must exceed stop_distance.'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_goal_backoff_m',
            default_value='0.45',
            description=(
                'Distance the rolling goal is kept back from the detected obstacle. '
                'Keep this above footprint + margin so the goal stays clear of walls.'
            ),
        ),
        DeclareLaunchArgument(
            'explorer_max_goal_distance_m',
            default_value='1.25',
            description='Maximum rolling-path look-ahead distance in metres.',
        ),
        DeclareLaunchArgument(
            'explorer_min_goal_distance_m',
            default_value='0.25',
            description='Minimum useful move when space is very tight.',
        ),
        DeclareLaunchArgument(
            'explorer_footprint_radius_m',
            default_value='0.30',
            description='Robot circular footprint radius used for clearance checks.',
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
            default_value='15.0',
            description=(
                'Half-width of the LiDAR sector used to evaluate a candidate '
                'heading.'
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
            'explorer_corner_mode_enabled',
            default_value='true',
            description='Enable corner-specific heading selection and target shaping.',
        ),
        DeclareLaunchArgument(
            'explorer_corner_forward_blocked_distance_m',
            default_value='0.72',
            description='Front clearance below which the explorer considers explicit corner handling.',
        ),
        DeclareLaunchArgument(
            'explorer_corner_side_open_distance_m',
            default_value='0.82',
            description='Side clearance required to treat left or right as the open corner direction.',
        ),
        DeclareLaunchArgument(
            'explorer_corner_clearance_window_deg',
            default_value='8.0',
            description='Narrow LiDAR window used when scoring headings through corners.',
        ),
        DeclareLaunchArgument(
            'explorer_corner_min_turn_angle_deg',
            default_value='35.0',
            description='Minimum turn angle considered for explicit corner handling.',
        ),
        DeclareLaunchArgument(
            'explorer_corner_max_turn_angle_deg',
            default_value='105.0',
            description='Maximum turn angle considered for explicit corner handling.',
        ),
        DeclareLaunchArgument(
            'explorer_corner_preferred_turn_angle_deg',
            default_value='70.0',
            description='Preferred corner turn angle used to score corner candidates.',
        ),
        DeclareLaunchArgument(
            'explorer_corner_goal_distance_m',
            default_value='0.28',
            description='Short rolling-path look-ahead used during corner mode.',
        ),
        DeclareLaunchArgument(
            'explorer_corner_outer_wall_bias_m',
            default_value='0.10',
            description='Lateral bias that pushes the corner target away from the inside wall.',
        ),
        DeclareLaunchArgument(
            'explorer_corner_commit_time_sec',
            default_value='1.0',
            description='How long corner-mode target shaping stays active after a corner heading is chosen.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_enabled',
            default_value='true',
            description='Enable explicit obstacle-bypass wall-follow behavior for convex local minima.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_trigger_replans',
            default_value='2',
            description='Consecutive low-progress replans before proactively preferring wall follow.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_front_blocked_distance_m',
            default_value='0.78',
            description='Front clearance below which wall-follow is considered a valid local-minimum escape.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_wall_detect_distance_m',
            default_value='0.95',
            description='Maximum distance at which a side obstacle boundary is considered followable.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_target_standoff_m',
            default_value='0.54',
            description='Desired wall clearance maintained during obstacle bypass.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_search_min_angle_deg',
            default_value='25.0',
            description='Minimum side-sector angle searched for a wall-follow boundary sample.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_search_max_angle_deg',
            default_value='155.0',
            description='Maximum side-sector angle searched for a wall-follow boundary sample.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_wall_window_deg',
            default_value='8.0',
            description='LiDAR window used when estimating the followed wall boundary.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_tangent_window_deg',
            default_value='10.0',
            description='LiDAR window used when checking tangent and follow headings.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_goal_distance_m',
            default_value='0.34',
            description='Rolling-path goal distance used while wall-follow is active.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_midpoint_distance_m',
            default_value='0.18',
            description='Intermediate waypoint distance used to shape wall-follow paths.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_goal_backoff_m',
            default_value='0.08',
            description='Distance kept behind the detected obstacle along the wall-follow heading.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_standoff_gain',
            default_value='1.15',
            description='Gain that pushes the wall-follow target away from or toward the boundary.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_forward_bias',
            default_value='0.30',
            description='Forward blend added to tangent motion so wall-follow keeps making progress.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_min_forward_component',
            default_value='0.25',
            description='Minimum forward component enforced on wall-follow motion vectors.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_max_correction_ratio',
            default_value='0.85',
            description='Maximum normalized standoff correction applied during wall-follow.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_min_tangent_clearance_m',
            default_value='0.66',
            description='Minimum tangent clearance required before a wall boundary is considered followable.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_max_follow_angle_deg',
            default_value='80.0',
            description='Maximum body-frame angle allowed for wall-follow translation.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_commit_time_sec',
            default_value='1.5',
            description='Minimum time to stay in wall-follow before considering an exit.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_cooldown_time_sec',
            default_value='1.0',
            description='Cooldown after leaving wall-follow to avoid immediate re-entry chatter.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_exit_forward_clearance_m',
            default_value='1.00',
            description='Front clearance required before leaving wall-follow for normal gap selection.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_exit_heading_max_deg',
            default_value='22.0',
            description='Maximum forward-heading error allowed when exiting wall-follow.',
        ),
        DeclareLaunchArgument(
            'explorer_wall_follow_stuck_time_sec',
            default_value='2.5',
            description='Time allowed for wall-follow to make progress before it is treated as stuck.',
        ),
        DeclareLaunchArgument(
            'explorer_max_replan_attempts',
            default_value='8',
            description='Consecutive low-travel stops before triggering a recovery backup.',
        ),
        DeclareLaunchArgument(
            'explorer_forward_bias_weight',
            default_value='1.5',
            description='Bias toward selecting the forward heading on startup; 0.0 disables.',
        ),
        DeclareLaunchArgument(
            'explorer_max_yaw_drift_deg',
            default_value='8.0',
            description='Maximum allowed odom yaw drift from a committed heading before forcing a replan.',
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
            'crab_follower_speed_mps',
            default_value='0.04',
            description='Constant travel speed for the crab path follower in m/s.',
        ),
        DeclareLaunchArgument(
            'crab_follower_goal_tolerance_m',
            default_value='0.08',
            description='Distance from the rolling goal at which the follower considers itself arrived.',
        ),
        DeclareLaunchArgument(
            'crab_follower_yaw_correction_gain',
            default_value='0.6',
            description='Proportional gain used to hold body yaw against drift while translating.',
        ),
        DeclareLaunchArgument(
            'crab_follower_max_angular_speed_rad_s',
            default_value='0.12',
            description='Maximum yaw-rate correction that the follower may command.',
        ),
        DeclareLaunchArgument(
            'crab_follower_yaw_deadband_deg',
            default_value='5.0',
            description='Yaw-error deadband that suppresses needless oscillation.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_yaw_offset_rad',
            default_value='0.0',
            description=(
                'Yaw offset between locomotion cmd_vel +X and the planner/base_link forward axis. '
                'Use this if the robot always moves with a constant left/right heading bias.'
            ),
        ),
        DeclareLaunchArgument(
            'safety_stop_distance_m',
            default_value='0.65',
            description='Emergency-stop clearance used by the lower-layer scan safety filter.',
        ),
        DeclareLaunchArgument(
            'safety_slowdown_distance_m',
            default_value='0.85',
            description='Clearance at which the lower-layer scan safety filter stops slowing translation.',
        ),
        DeclareLaunchArgument(
            'safety_clearance_window_deg',
            default_value='15.0',
            description='Half-width of the LiDAR sector checked by the lower-layer scan safety filter.',
        ),
        DeclareLaunchArgument(
            'safety_side_stop_distance_m',
            default_value='0.32',
            description=(
                'Minimum allowed LiDAR clearance at the robot flanks before side-wall '
                'safety blocks translation.'
            ),
        ),
        DeclareLaunchArgument(
            'safety_side_slowdown_distance_m',
            default_value='0.46',
            description=(
                'Side-wall clearance at which the lower-layer safety filter stops '
                'slowing or nudging away from nearby walls.'
            ),
        ),
        DeclareLaunchArgument(
            'safety_side_clearance_window_deg',
            default_value='20.0',
            description=(
                'Half-width of the LiDAR sectors checked to the left and right of '
                'the current motion direction.'
            ),
        ),
        DeclareLaunchArgument(
            'safety_max_side_push_ratio',
            default_value='0.75',
            description=(
                'Maximum lateral blend the safety filter may add to bias motion away '
                'from a nearby side wall.'
            ),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '--x', laser_x, '--y', laser_y, '--z', laser_z,
                '--roll', laser_roll, '--pitch', laser_pitch, '--yaw', laser_yaw,
                '--frame-id', base_frame, '--child-frame-id', laser_frame,
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(enable_robot_localization),
            parameters=[
                robot_localization_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'odom_frame': odom_frame,
                    'base_link_frame': base_frame,
                    'world_frame': odom_frame,
                    'odom0': raw_odom_topic,
                    'imu0': imu_topic,
                },
            ],
            remappings=[
                ('odometry/filtered', odom_topic),
            ],
        ),
        slam_toolbox_node,
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=IfCondition(autostart_slam),
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox_node,
                goal_state='inactive',
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_toolbox_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        ),
                    ),
                ],
            ),
            condition=IfCondition(autostart_slam),
        ),
        Node(
            package='hexapod_slam',
            executable='gap_following_explorer',
            name='gap_following_explorer',
            output='screen',
            condition=IfCondition(enable_explorer),
            parameters=[{
                'scan_topic': scan_topic,
                'scan_yaw_offset_rad': laser_yaw,
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
                'corner_mode_enabled': explorer_corner_mode_enabled,
                'corner_forward_blocked_distance_m': explorer_corner_forward_blocked_distance_m,
                'corner_side_open_distance_m': explorer_corner_side_open_distance_m,
                'corner_clearance_window_deg': explorer_corner_clearance_window_deg,
                'corner_min_turn_angle_deg': explorer_corner_min_turn_angle_deg,
                'corner_max_turn_angle_deg': explorer_corner_max_turn_angle_deg,
                'corner_preferred_turn_angle_deg': explorer_corner_preferred_turn_angle_deg,
                'corner_goal_distance_m': explorer_corner_goal_distance_m,
                'corner_outer_wall_bias_m': explorer_corner_outer_wall_bias_m,
                'corner_commit_time_sec': explorer_corner_commit_time_sec,
                'wall_follow_enabled': explorer_wall_follow_enabled,
                'wall_follow_trigger_replans': explorer_wall_follow_trigger_replans,
                'wall_follow_front_blocked_distance_m': explorer_wall_follow_front_blocked_distance_m,
                'wall_follow_wall_detect_distance_m': explorer_wall_follow_wall_detect_distance_m,
                'wall_follow_target_standoff_m': explorer_wall_follow_target_standoff_m,
                'wall_follow_search_min_angle_deg': explorer_wall_follow_search_min_angle_deg,
                'wall_follow_search_max_angle_deg': explorer_wall_follow_search_max_angle_deg,
                'wall_follow_wall_window_deg': explorer_wall_follow_wall_window_deg,
                'wall_follow_tangent_window_deg': explorer_wall_follow_tangent_window_deg,
                'wall_follow_goal_distance_m': explorer_wall_follow_goal_distance_m,
                'wall_follow_midpoint_distance_m': explorer_wall_follow_midpoint_distance_m,
                'wall_follow_goal_backoff_m': explorer_wall_follow_goal_backoff_m,
                'wall_follow_standoff_gain': explorer_wall_follow_standoff_gain,
                'wall_follow_forward_bias': explorer_wall_follow_forward_bias,
                'wall_follow_min_forward_component': explorer_wall_follow_min_forward_component,
                'wall_follow_max_correction_ratio': explorer_wall_follow_max_correction_ratio,
                'wall_follow_min_tangent_clearance_m': explorer_wall_follow_min_tangent_clearance_m,
                'wall_follow_max_follow_angle_deg': explorer_wall_follow_max_follow_angle_deg,
                'wall_follow_commit_time_sec': explorer_wall_follow_commit_time_sec,
                'wall_follow_cooldown_time_sec': explorer_wall_follow_cooldown_time_sec,
                'wall_follow_exit_forward_clearance_m': explorer_wall_follow_exit_forward_clearance_m,
                'wall_follow_exit_heading_max_deg': explorer_wall_follow_exit_heading_max_deg,
                'wall_follow_stuck_time_sec': explorer_wall_follow_stuck_time_sec,
                'max_replan_attempts': explorer_max_replan_attempts,
                'forward_bias_weight': explorer_forward_bias_weight,
                'max_yaw_drift_deg': explorer_max_yaw_drift_deg,
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
                'path_topic': path_topic,
                'odom_topic': odom_topic,
                'cmd_vel_topic': raw_cmd_vel_topic,
                'constant_speed_mps': crab_follower_speed_mps,
                'goal_tolerance_m': crab_follower_goal_tolerance_m,
                'path_timeout_sec': 1.0,
                'cmd_vel_rate_hz': 20.0,
                'yaw_correction_gain': crab_follower_yaw_correction_gain,
                'max_angular_speed_rad_s': crab_follower_max_angular_speed_rad_s,
                'yaw_deadband_deg': crab_follower_yaw_deadband_deg,
                'cmd_vel_yaw_offset_rad': cmd_vel_yaw_offset_rad,
            }],
        ),
        Node(
            package='hexapod_slam',
            executable='scan_cmd_vel_safety',
            name='scan_cmd_vel_safety',
            output='screen',
            condition=IfCondition(enable_explorer),
            parameters=[{
                'scan_topic': scan_topic,
                'scan_yaw_offset_rad': laser_yaw,
                'cmd_vel_yaw_offset_rad': cmd_vel_yaw_offset_rad,
                'input_cmd_vel_topic': raw_cmd_vel_topic,
                'output_cmd_vel_topic': safe_cmd_vel_topic,
                'control_rate_hz': 20.0,
                'scan_timeout_sec': 0.5,
                'cmd_timeout_sec': 0.5,
                'clearance_window_deg': safety_clearance_window_deg,
                'stop_distance_m': safety_stop_distance_m,
                'slowdown_distance_m': safety_slowdown_distance_m,
                'side_clearance_window_deg': safety_side_clearance_window_deg,
                'side_stop_distance_m': safety_side_stop_distance_m,
                'side_slowdown_distance_m': safety_side_slowdown_distance_m,
                'max_side_push_ratio': safety_max_side_push_ratio,
                'preserve_turning_when_blocked': False,
            }],
        ),
    ])
