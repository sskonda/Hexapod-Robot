#!/usr/bin/env python3
"""
maze_mission.launch.py

Launches the full Hexapod Mission 1: Maze Traversal stack:
  - static_transform_publisher  (base_link → laser)
  - slam_toolbox                (optional, default enabled)
  - maze_mission                (grid-graph DFS/BFS traversal)
  - crab_path_follower          (translates Path → cmd_vel)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('hexapod_slam')
    slam_params = PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox.yaml'])

    # -----------------------------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------------------------
    args = [
        DeclareLaunchArgument('use_slam', default_value='true',
                              description='Launch slam_toolbox alongside maze_mission.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock (simulation) time.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='LaserScan topic.'),
        DeclareLaunchArgument('odom_topic', default_value='odom',
                              description='Odometry topic.'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                              description='Robot base TF frame.'),
        DeclareLaunchArgument('laser_frame', default_value='laser',
                              description='LiDAR TF frame — must match /scan frame_id.'),
        DeclareLaunchArgument('laser_x', default_value='0.0',
                              description='LiDAR X offset from base_link (m).'),
        DeclareLaunchArgument('laser_y', default_value='0.0',
                              description='LiDAR Y offset from base_link (m).'),
        DeclareLaunchArgument('laser_z', default_value='0.0',
                              description='LiDAR Z offset from base_link (m).'),
        DeclareLaunchArgument('use_bfs', default_value='false',
                              description='true=BFS breadth-first, false=DFS left-first.'),
        DeclareLaunchArgument('wall_threshold_m', default_value='0.38',
                              description='Clearance < this → wall present.'),
        DeclareLaunchArgument('open_threshold_m', default_value='0.55',
                              description='Clearance > this → open passage.'),
        DeclareLaunchArgument('exit_threshold_m', default_value='1.2192',
                              description='Clearance > this in ≥2 dirs → exit (2×tile).'),
        DeclareLaunchArgument('centering_tolerance_m', default_value='0.05',
                              description='Tolerance to consider robot at node centre.'),
        DeclareLaunchArgument('classification_scans', default_value='5',
                              description='Number of scans that must agree for wall classification.'),
        DeclareLaunchArgument('classification_ratio', default_value='0.8',
                              description='Fraction of scans that must agree.'),
        DeclareLaunchArgument('traversal_speed_mps', default_value='0.04',
                              description='Expected robot speed (used for timeout calculation).'),
        DeclareLaunchArgument('traversal_timeout_factor', default_value='3.0',
                              description='Timeout = factor × (tile_size / speed).'),
        DeclareLaunchArgument('scan_window_deg', default_value='15.0',
                              description='Half-window for clearance measurement (deg).'),
        DeclareLaunchArgument('graph_save_path', default_value='/tmp/maze_graph.json',
                              description='Path to save the JSON graph on exit.'),
        DeclareLaunchArgument('crab_speed_mps', default_value='0.04',
                              description='Speed for crab_path_follower (m/s).'),
        DeclareLaunchArgument('crab_goal_tolerance_m', default_value='0.05',
                              description='Goal tolerance for crab_path_follower (m).'),
        DeclareLaunchArgument('recovery_backup_m', default_value='0.20',
                              description='Recovery backup distance (m).'),
    ]

    use_slam         = LaunchConfiguration('use_slam')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    scan_topic       = LaunchConfiguration('scan_topic')
    odom_topic       = LaunchConfiguration('odom_topic')
    base_frame       = LaunchConfiguration('base_frame')
    laser_frame      = LaunchConfiguration('laser_frame')
    laser_x          = LaunchConfiguration('laser_x')
    laser_y          = LaunchConfiguration('laser_y')
    laser_z          = LaunchConfiguration('laser_z')
    use_bfs          = LaunchConfiguration('use_bfs')
    wall_threshold_m = LaunchConfiguration('wall_threshold_m')
    open_threshold_m = LaunchConfiguration('open_threshold_m')
    exit_threshold_m = LaunchConfiguration('exit_threshold_m')
    centering_tol    = LaunchConfiguration('centering_tolerance_m')
    cls_scans        = LaunchConfiguration('classification_scans')
    cls_ratio        = LaunchConfiguration('classification_ratio')
    trav_speed       = LaunchConfiguration('traversal_speed_mps')
    trav_factor      = LaunchConfiguration('traversal_timeout_factor')
    scan_window      = LaunchConfiguration('scan_window_deg')
    graph_path       = LaunchConfiguration('graph_save_path')
    crab_speed       = LaunchConfiguration('crab_speed_mps')
    crab_tol         = LaunchConfiguration('crab_goal_tolerance_m')
    recovery_m       = LaunchConfiguration('recovery_backup_m')

    # -----------------------------------------------------------------------
    # static_transform_publisher: base_link → laser
    # -----------------------------------------------------------------------
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=[
            '--x', laser_x, '--y', laser_y, '--z', laser_z,
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', base_frame, '--child-frame-id', laser_frame,
        ],
    )

    # -----------------------------------------------------------------------
    # slam_toolbox (optional)
    # -----------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            ])
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_slam),
    )

    # -----------------------------------------------------------------------
    # maze_mission node
    # -----------------------------------------------------------------------
    maze_mission_node = Node(
        package='hexapod_slam',
        executable='maze_mission',
        name='maze_mission',
        output='screen',
        parameters=[{
            'scan_topic':             scan_topic,
            'odom_topic':             odom_topic,
            'use_sim_time':           use_sim_time,
            'use_bfs':                use_bfs,
            'wall_threshold_m':       wall_threshold_m,
            'open_threshold_m':       open_threshold_m,
            'exit_threshold_m':       exit_threshold_m,
            'centering_tolerance_m':  centering_tol,
            'classification_scans':   cls_scans,
            'classification_ratio':   cls_ratio,
            'traversal_speed_mps':    trav_speed,
            'traversal_timeout_factor': trav_factor,
            'scan_window_deg':        scan_window,
            'graph_save_path':        graph_path,
            'recovery_backup_m':      recovery_m,
        }],
    )

    # -----------------------------------------------------------------------
    # crab_path_follower (from hexapod_locomotion)
    # -----------------------------------------------------------------------
    crab_follower_node = Node(
        package='hexapod_locomotion',
        executable='crab_path_follower',
        name='crab_path_follower',
        output='screen',
        parameters=[{
            'path_topic':         'maze_mission/path',
            'odom_topic':         odom_topic,
            'cmd_vel_topic':      'cmd_vel',
            'constant_speed_mps': crab_speed,
            'goal_tolerance_m':   crab_tol,
            'path_timeout_sec':   0.5,
            'cmd_vel_rate_hz':    20.0,
        }],
    )

    return LaunchDescription(args + [
        static_tf_node,
        slam_launch,
        maze_mission_node,
        crab_follower_node,
    ])
