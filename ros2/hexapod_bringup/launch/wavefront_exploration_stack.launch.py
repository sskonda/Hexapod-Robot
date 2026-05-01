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
    active_goal_topic = LaunchConfiguration('active_goal_topic')
    planned_path_topic = LaunchConfiguration('planned_path_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    locomotion_use_imu_for_odom = LaunchConfiguration('locomotion_use_imu_for_odom')
    show_imu_data = LaunchConfiguration('show_imu_data')

    explorer_enabled = LaunchConfiguration('explorer_enabled')
    explorer_replan_period_sec = LaunchConfiguration('explorer_replan_period_sec')
    explorer_free_threshold = LaunchConfiguration('explorer_free_threshold')
    explorer_occupied_threshold = LaunchConfiguration('explorer_occupied_threshold')
    explorer_robot_radius_m = LaunchConfiguration('explorer_robot_radius_m')
    explorer_safety_margin_m = LaunchConfiguration('explorer_safety_margin_m')
    explorer_min_frontier_cluster_size = LaunchConfiguration(
        'explorer_min_frontier_cluster_size'
    )
    explorer_goal_projection_radius_m = LaunchConfiguration(
        'explorer_goal_projection_radius_m'
    )
    explorer_goal_tolerance_m = LaunchConfiguration('explorer_goal_tolerance_m')
    explorer_waypoint_spacing_m = LaunchConfiguration('explorer_waypoint_spacing_m')
    explorer_max_speed_mps = LaunchConfiguration('explorer_max_speed_mps')
    explorer_max_turn_rate_rad_s = LaunchConfiguration('explorer_max_turn_rate_rad_s')
    explorer_publish_target_markers = LaunchConfiguration('explorer_publish_target_markers')
    explorer_target_marker_scale_m = LaunchConfiguration('explorer_target_marker_scale_m')

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
            description='LaserScan topic passed to slam_toolbox.',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='OccupancyGrid topic consumed by wavefront exploration.',
        ),
        DeclareLaunchArgument(
            'target_marker_topic',
            default_value='/explorer/targets',
            description='MarkerArray topic for RViz WFD diagnostics.',
        ),
        DeclareLaunchArgument(
            'active_goal_topic',
            default_value='/explorer/active_goal',
            description='PoseStamped topic for the selected safe exploration goal.',
        ),
        DeclareLaunchArgument(
            'planned_path_topic',
            default_value='/explorer/planned_path',
            description='Path topic for the selected reachable route.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel',
            description='Velocity command topic used by the wavefront explorer.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot body frame used for the current WFD start cell.',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame used by wavefront exploration.',
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
            description='Pass through to pose_stack.launch.py.',
        ),
        DeclareLaunchArgument(
            'explorer_enabled',
            default_value='true',
            description='When false, the wavefront explorer publishes stop commands only.',
        ),
        DeclareLaunchArgument(
            'explorer_replan_period_sec',
            default_value='3.0',
            description='How often to rerun Wavefront Frontier Detection.',
        ),
        DeclareLaunchArgument(
            'explorer_free_threshold',
            default_value='25',
            description='Occupancy values from 0 through this value are FREE.',
        ),
        DeclareLaunchArgument(
            'explorer_occupied_threshold',
            default_value='65',
            description='Occupancy values at or above this value are BLOCKED.',
        ),
        DeclareLaunchArgument(
            'explorer_robot_radius_m',
            default_value='0.30',
            description='Robot radius used for BLOCKED-cell inflation.',
        ),
        DeclareLaunchArgument(
            'explorer_safety_margin_m',
            default_value='0.10',
            description='Extra safety inflation added around BLOCKED cells.',
        ),
        DeclareLaunchArgument(
            'explorer_min_frontier_cluster_size',
            default_value='5',
            description='Reject frontier clusters smaller than this many cells.',
        ),
        DeclareLaunchArgument(
            'explorer_goal_projection_radius_m',
            default_value='0.90',
            description='Search radius for safe reachable goal projection.',
        ),
        DeclareLaunchArgument(
            'explorer_goal_tolerance_m',
            default_value='0.18',
            description='Distance at which the selected goal is considered reached.',
        ),
        DeclareLaunchArgument(
            'explorer_waypoint_spacing_m',
            default_value='0.25',
            description='Spacing used when following the selected WFD path.',
        ),
        DeclareLaunchArgument(
            'explorer_max_speed_mps',
            default_value='0.035',
            description='Maximum forward speed while following the WFD path.',
        ),
        DeclareLaunchArgument(
            'explorer_max_turn_rate_rad_s',
            default_value='0.18',
            description='Maximum turn rate while following the WFD path.',
        ),
        DeclareLaunchArgument(
            'explorer_publish_target_markers',
            default_value='true',
            description='Publish RViz markers for frontiers, clusters, goal, and path.',
        ),
        DeclareLaunchArgument(
            'explorer_target_marker_scale_m',
            default_value='0.12',
            description='Marker scale for selected goal and cluster centroid markers.',
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
            executable='wavefront_explorer',
            name='wavefront_explorer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_topic': map_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'target_marker_topic': target_marker_topic,
                'active_goal_topic': active_goal_topic,
                'planned_path_topic': planned_path_topic,
                'base_frame': base_frame,
                'map_frame': map_frame,
                'enabled': explorer_enabled,
                'replan_period_sec': explorer_replan_period_sec,
                'free_threshold': explorer_free_threshold,
                'occupied_threshold': explorer_occupied_threshold,
                'robot_radius_m': explorer_robot_radius_m,
                'safety_margin_m': explorer_safety_margin_m,
                'min_frontier_cluster_size': explorer_min_frontier_cluster_size,
                'goal_projection_radius_m': explorer_goal_projection_radius_m,
                'goal_tolerance_m': explorer_goal_tolerance_m,
                'waypoint_spacing_m': explorer_waypoint_spacing_m,
                'max_speed_mps': explorer_max_speed_mps,
                'max_turn_rate_rad_s': explorer_max_turn_rate_rad_s,
                'publish_target_markers': explorer_publish_target_markers,
                'target_marker_scale_m': explorer_target_marker_scale_m,
            }],
        ),
    ])
