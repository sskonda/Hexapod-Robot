from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share_dir = Path(get_package_share_directory('hexapod_bringup'))
    pose_stack_launch = bringup_share_dir / 'launch' / 'pose_stack.launch.py'

    servo_dry_run = LaunchConfiguration('servo_dry_run')
    launch_lidar = LaunchConfiguration('launch_lidar')
    enable_slam_toolbox = LaunchConfiguration('enable_slam_toolbox')
    scan_topic = LaunchConfiguration('scan_topic')
    map_topic = LaunchConfiguration('map_topic')
    target_marker_topic = LaunchConfiguration('target_marker_topic')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    locomotion_use_imu_for_odom = LaunchConfiguration('locomotion_use_imu_for_odom')
    show_imu_data = LaunchConfiguration('show_imu_data')

    explorer_enabled = LaunchConfiguration('explorer_enabled')
    explorer_mode = LaunchConfiguration('explorer_mode')
    strategy = LaunchConfiguration('strategy')
    explorer_reactive_fallback = LaunchConfiguration('explorer_reactive_fallback')
    explorer_max_speed_mps = LaunchConfiguration('explorer_max_speed_mps')
    explorer_min_speed_mps = LaunchConfiguration('explorer_min_speed_mps')
    explorer_obstacle_stop_distance_m = LaunchConfiguration(
        'explorer_obstacle_stop_distance_m'
    )
    explorer_obstacle_slow_distance_m = LaunchConfiguration(
        'explorer_obstacle_slow_distance_m'
    )
    explorer_side_stop_distance_m = LaunchConfiguration('explorer_side_stop_distance_m')
    explorer_desired_clearance_m = LaunchConfiguration('explorer_desired_clearance_m')
    explorer_crab_motion = LaunchConfiguration('explorer_crab_motion')
    explorer_reverse_allowed = LaunchConfiguration('explorer_reverse_allowed')
    explorer_use_tf_for_scan_frame = LaunchConfiguration('explorer_use_tf_for_scan_frame')
    explorer_scan_yaw_offset_deg = LaunchConfiguration('explorer_scan_yaw_offset_deg')
    explorer_frontier_replan_period_sec = LaunchConfiguration(
        'explorer_frontier_replan_period_sec'
    )
    explorer_frontier_goal_tolerance_m = LaunchConfiguration(
        'explorer_frontier_goal_tolerance_m'
    )
    explorer_frontier_waypoint_spacing_m = LaunchConfiguration(
        'explorer_frontier_waypoint_spacing_m'
    )
    explorer_frontier_min_clearance_m = LaunchConfiguration(
        'explorer_frontier_min_clearance_m'
    )
    explorer_robot_radius_m = LaunchConfiguration('explorer_robot_radius_m')
    explorer_planner_safety_margin_m = LaunchConfiguration(
        'explorer_planner_safety_margin_m'
    )
    explorer_frontier_unknown_margin_cells = LaunchConfiguration(
        'explorer_frontier_unknown_margin_cells'
    )
    explorer_frontier_min_area_cells = LaunchConfiguration(
        'explorer_frontier_min_area_cells'
    )
    explorer_frontier_goal_projection_radius_m = LaunchConfiguration(
        'explorer_frontier_goal_projection_radius_m'
    )
    explorer_frontier_standoff_distance_m = LaunchConfiguration(
        'explorer_frontier_standoff_distance_m'
    )
    explorer_frontier_failure_memory_enabled = LaunchConfiguration(
        'explorer_frontier_failure_memory_enabled'
    )
    explorer_frontier_suppression_duration_sec = LaunchConfiguration(
        'explorer_frontier_suppression_duration_sec'
    )
    explorer_frontier_suppression_radius_m = LaunchConfiguration(
        'explorer_frontier_suppression_radius_m'
    )
    explorer_frontier_blocked_clearance_margin_m = LaunchConfiguration(
        'explorer_frontier_blocked_clearance_margin_m'
    )
    explorer_frontier_progress_timeout_sec = LaunchConfiguration(
        'explorer_frontier_progress_timeout_sec'
    )
    explorer_frontier_progress_epsilon_m = LaunchConfiguration(
        'explorer_frontier_progress_epsilon_m'
    )
    explorer_publish_target_markers = LaunchConfiguration('explorer_publish_target_markers')
    explorer_target_marker_scale_m = LaunchConfiguration('explorer_target_marker_scale_m')
    explorer_bug_recovery_enabled = LaunchConfiguration('explorer_bug_recovery_enabled')
    explorer_bug_wall_side = LaunchConfiguration('explorer_bug_wall_side')
    explorer_bug_forward_speed_mps = LaunchConfiguration('explorer_bug_forward_speed_mps')
    explorer_bug_desired_wall_distance_m = LaunchConfiguration(
        'explorer_bug_desired_wall_distance_m'
    )
    explorer_bug_release_clearance_m = LaunchConfiguration('explorer_bug_release_clearance_m')
    explorer_bug_min_duration_sec = LaunchConfiguration('explorer_bug_min_duration_sec')
    explorer_bug_max_duration_sec = LaunchConfiguration('explorer_bug_max_duration_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'servo_dry_run',
            default_value='true',
            description='Default true so the first exploration launch is a no-motion test.',
        ),
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Launch the RPLIDAR driver.',
        ),
        DeclareLaunchArgument(
            'enable_slam_toolbox',
            default_value='true',
            description='Launch slam_toolbox for 2D mapping.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic used by the explorer and slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='OccupancyGrid topic used by frontier exploration.',
        ),
        DeclareLaunchArgument(
            'target_marker_topic',
            default_value='/explorer/targets',
            description='MarkerArray topic for RViz target nodes and frontier path.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot body frame used by the explorer for cmd_vel directions.',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame used by frontier exploration.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if a simulator provides /clock.',
        ),
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device for the RPLIDAR A1 USB adapter.',
        ),
        DeclareLaunchArgument(
            'locomotion_use_imu_for_odom',
            default_value='false',
            description='Pass through to pose_stack.launch.py.',
        ),
        DeclareLaunchArgument(
            'show_imu_data',
            default_value='true',
            description='When false, suppress routine BNO055 IMU status logs while still publishing IMU topics.',
        ),
        DeclareLaunchArgument(
            'explorer_enabled',
            default_value='true',
            description='When false, explorer publishes stop commands only.',
        ),
        DeclareLaunchArgument(
            'explorer_mode',
            default_value='frontier',
            description='Explorer mode: "frontier" uses /map, "reactive" uses only /scan.',
        ),
        DeclareLaunchArgument(
            'strategy',
            default_value='bfs',
            description='Frontier search strategy: "bfs" maps nearest frontiers first, "dfs" dives deeper.',
        ),
        DeclareLaunchArgument(
            'explorer_reactive_fallback',
            default_value='true',
            description='Fall back to scan-reactive motion when no usable frontier target is available.',
        ),
        DeclareLaunchArgument(
            'explorer_max_speed_mps',
            default_value='0.035',
            description='Maximum exploration translation speed.',
        ),
        DeclareLaunchArgument(
            'explorer_min_speed_mps',
            default_value='0.012',
            description='Minimum speed when moving through a narrowing opening.',
        ),
        DeclareLaunchArgument(
            'explorer_obstacle_stop_distance_m',
            default_value='0.30',
            description='Stop/turn if the selected direction is closer than this.',
        ),
        DeclareLaunchArgument(
            'explorer_obstacle_slow_distance_m',
            default_value='0.60',
            description='Start slowing below this selected-direction clearance.',
        ),
        DeclareLaunchArgument(
            'explorer_side_stop_distance_m',
            default_value='0.38',
            description='Minimum allowed left/right wall clearance before sliding away.',
        ),
        DeclareLaunchArgument(
            'explorer_desired_clearance_m',
            default_value='0.45',
            description='Range threshold used to compute angular clearance.',
        ),
        DeclareLaunchArgument(
            'explorer_crab_motion',
            default_value='true',
            description='Use x/y crab motion toward the chosen LiDAR direction.',
        ),
        DeclareLaunchArgument(
            'explorer_reverse_allowed',
            default_value='false',
            description='Allow the explorer to command negative linear.x.',
        ),
        DeclareLaunchArgument(
            'explorer_use_tf_for_scan_frame',
            default_value='true',
            description='Use TF to rotate scan directions into base_frame before publishing cmd_vel.',
        ),
        DeclareLaunchArgument(
            'explorer_scan_yaw_offset_deg',
            default_value='0.0',
            description='Fallback scan-to-body yaw offset if TF is unavailable.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_replan_period_sec',
            default_value='5.0',
            description='How often to re-run BFS/DFS frontier selection.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_goal_tolerance_m',
            default_value='0.18',
            description='Distance at which a frontier target is considered reached.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_waypoint_spacing_m',
            default_value='0.25',
            description='Spacing between followed waypoints on the BFS/DFS map path.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_min_clearance_m',
            default_value='0.15',
            description='Minimum map clearance from occupied cells for frontier goals and traversed cells.',
        ),
        DeclareLaunchArgument(
            'explorer_robot_radius_m',
            default_value='0.30',
            description='Robot body radius used for occupancy-grid obstacle inflation.',
        ),
        DeclareLaunchArgument(
            'explorer_planner_safety_margin_m',
            default_value='0.10',
            description='Extra inflation margin added around occupied/unknown-danger cells.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_unknown_margin_cells',
            default_value='2',
            description='Unknown cells this many cells from occupied cells are treated as danger.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_min_area_cells',
            default_value='5',
            description='Reject connected frontier regions smaller than this many cells.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_goal_projection_radius_m',
            default_value='0.90',
            description='Search radius for projecting frontier boundaries to safe standoff goals.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_standoff_distance_m',
            default_value='0.40',
            description='Preferred distance back into known free space from a frontier boundary.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_failure_memory_enabled',
            default_value='true',
            description='Remember failed frontier paths and temporarily avoid repeating them.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_suppression_duration_sec',
            default_value='45.0',
            description='How long to avoid a frontier after reaching it or rejecting its current path.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_suppression_radius_m',
            default_value='0.75',
            description='Radius around a rejected frontier to skip during the suppression window.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_blocked_clearance_margin_m',
            default_value='0.05',
            description='Extra clearance margin that treats a frontier path as blocked before stop-distance jitter.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_progress_timeout_sec',
            default_value='10.0',
            description='Seconds without getting closer to a waypoint before Bug recovery or suppression.',
        ),
        DeclareLaunchArgument(
            'explorer_frontier_progress_epsilon_m',
            default_value='0.08',
            description='Minimum distance improvement that counts as progress toward a waypoint.',
        ),
        DeclareLaunchArgument(
            'explorer_publish_target_markers',
            default_value='true',
            description='Publish RViz markers for frontier target nodes and the active local target.',
        ),
        DeclareLaunchArgument(
            'explorer_target_marker_scale_m',
            default_value='0.12',
            description='Marker size for RViz frontier target nodes.',
        ),
        DeclareLaunchArgument(
            'explorer_bug_recovery_enabled',
            default_value='true',
            description='Use simple Bug-style wall following when the direct frontier waypoint is blocked.',
        ),
        DeclareLaunchArgument(
            'explorer_bug_wall_side',
            default_value='auto',
            description='Bug wall-follow side: "auto", "left", or "right".',
        ),
        DeclareLaunchArgument(
            'explorer_bug_forward_speed_mps',
            default_value='0.012',
            description='Forward speed while circumnavigating an obstacle in Bug recovery.',
        ),
        DeclareLaunchArgument(
            'explorer_bug_desired_wall_distance_m',
            default_value='0.40',
            description='Approximate side distance to hold from the followed wall during Bug recovery.',
        ),
        DeclareLaunchArgument(
            'explorer_bug_release_clearance_m',
            default_value='0.55',
            description='Direct waypoint clearance needed before leaving Bug recovery.',
        ),
        DeclareLaunchArgument(
            'explorer_bug_min_duration_sec',
            default_value='3.0',
            description='Minimum time to stay in Bug recovery before direct-path release is allowed.',
        ),
        DeclareLaunchArgument(
            'explorer_bug_max_duration_sec',
            default_value='15.0',
            description='Maximum time spent in Bug recovery before rejecting the current frontier path.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pose_stack_launch)),
            launch_arguments={
                'servo_dry_run': servo_dry_run,
                'launch_lidar': launch_lidar,
                'enable_slam_toolbox': enable_slam_toolbox,
                'scan_topic': scan_topic,
                'use_sim_time': use_sim_time,
                'map_frame': map_frame,
                'lidar_serial_port': lidar_serial_port,
                'locomotion_use_imu_for_odom': locomotion_use_imu_for_odom,
                'show_imu_data': show_imu_data,
            }.items(),
        ),
        Node(
            package='hexapod_locomotion',
            executable='lidar_open_space_explorer',
            name='lidar_open_space_explorer',
            output='screen',
            parameters=[{
                'scan_topic': scan_topic,
                'map_topic': map_topic,
                'target_marker_topic': target_marker_topic,
                'cmd_vel_topic': 'cmd_vel',
                'base_frame': base_frame,
                'map_frame': map_frame,
                'enabled': explorer_enabled,
                'exploration_mode': explorer_mode,
                'search_strategy': strategy,
                'reactive_fallback': explorer_reactive_fallback,
                'use_tf_for_scan_frame': explorer_use_tf_for_scan_frame,
                'scan_yaw_offset_deg': explorer_scan_yaw_offset_deg,
                'max_speed_mps': explorer_max_speed_mps,
                'min_speed_mps': explorer_min_speed_mps,
                'obstacle_stop_distance_m': explorer_obstacle_stop_distance_m,
                'obstacle_slow_distance_m': explorer_obstacle_slow_distance_m,
                'side_stop_distance_m': explorer_side_stop_distance_m,
                'desired_clearance_m': explorer_desired_clearance_m,
                'crab_motion': explorer_crab_motion,
                'reverse_allowed': explorer_reverse_allowed,
                'frontier_replan_period_sec': explorer_frontier_replan_period_sec,
                'frontier_goal_tolerance_m': explorer_frontier_goal_tolerance_m,
                'frontier_waypoint_spacing_m': explorer_frontier_waypoint_spacing_m,
                'frontier_min_clearance_m': explorer_frontier_min_clearance_m,
                'robot_radius_m': explorer_robot_radius_m,
                'planner_safety_margin_m': explorer_planner_safety_margin_m,
                'frontier_unknown_margin_cells': explorer_frontier_unknown_margin_cells,
                'frontier_min_area_cells': explorer_frontier_min_area_cells,
                'frontier_goal_projection_radius_m': (
                    explorer_frontier_goal_projection_radius_m
                ),
                'frontier_standoff_distance_m': explorer_frontier_standoff_distance_m,
                'frontier_failure_memory_enabled': (
                    explorer_frontier_failure_memory_enabled
                ),
                'frontier_suppression_duration_sec': (
                    explorer_frontier_suppression_duration_sec
                ),
                'frontier_suppression_radius_m': explorer_frontier_suppression_radius_m,
                'frontier_blocked_clearance_margin_m': (
                    explorer_frontier_blocked_clearance_margin_m
                ),
                'frontier_progress_timeout_sec': explorer_frontier_progress_timeout_sec,
                'frontier_progress_epsilon_m': explorer_frontier_progress_epsilon_m,
                'publish_target_markers': explorer_publish_target_markers,
                'target_marker_scale_m': explorer_target_marker_scale_m,
                'bug_recovery_enabled': explorer_bug_recovery_enabled,
                'bug_wall_side': explorer_bug_wall_side,
                'bug_forward_speed_mps': explorer_bug_forward_speed_mps,
                'bug_desired_wall_distance_m': explorer_bug_desired_wall_distance_m,
                'bug_release_clearance_m': explorer_bug_release_clearance_m,
                'bug_min_duration_sec': explorer_bug_min_duration_sec,
                'bug_max_duration_sec': explorer_bug_max_duration_sec,
            }],
        ),
    ])
