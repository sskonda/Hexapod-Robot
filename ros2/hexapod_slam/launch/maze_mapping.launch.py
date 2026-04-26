#!/usr/bin/env python3
"""
maze_mapping.launch.py

Launches the full maze-mapping and topological exploration stack:
  - slam_toolbox (optional)
  - maze_graph_builder
  - maze_graph_planner
  - maze_graph_visualizer
  - path_to_local_goal
  - gap_following_explorer (optional)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("hexapod_slam")

    # -----------------------------------------------------------------------
    # Launch arguments
    # -----------------------------------------------------------------------
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true",
        description="Launch slam_toolbox alongside the graph stack.",
    )
    use_locomotion_arg = DeclareLaunchArgument(
        "use_locomotion",
        default_value="false",
        description="Launch crab_path_follower + scan_cmd_vel_safety on /maze_graph/path.",
    )
    use_explorer_arg = DeclareLaunchArgument(
        "use_explorer",
        default_value="false",
        description="Launch gap_following_explorer for local obstacle avoidance.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock (simulation) time.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="odom",
        description="Odometry topic for the graph planner and path follower.",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="LaserScan topic for the safety filter.",
    )
    raw_cmd_vel_topic_arg = DeclareLaunchArgument(
        "raw_cmd_vel_topic",
        default_value="cmd_vel_nav",
        description="Intermediate cmd_vel topic before scan safety filtering.",
    )
    safe_cmd_vel_topic_arg = DeclareLaunchArgument(
        "safe_cmd_vel_topic",
        default_value="cmd_vel",
        description="Final cmd_vel topic after scan safety filtering.",
    )
    cmd_vel_yaw_offset_arg = DeclareLaunchArgument(
        "cmd_vel_yaw_offset_rad",
        default_value="1.5708",
        description="Yaw offset between locomotion cmd_vel +X and planner forward.",
    )
    crab_follower_speed_arg = DeclareLaunchArgument(
        "crab_follower_speed_mps",
        default_value="0.04",
        description="Crab follower translation speed when following /maze_graph/path.",
    )
    crab_follower_goal_tolerance_arg = DeclareLaunchArgument(
        "crab_follower_goal_tolerance_m",
        default_value="0.10",
        description="Distance at which the crab follower considers a graph waypoint reached.",
    )
    crab_follower_yaw_ki_arg = DeclareLaunchArgument(
        "crab_follower_yaw_ki",
        default_value="0.0",
        description="Integral gain used by the crab follower yaw hold controller.",
    )
    crab_follower_yaw_integrator_limit_arg = DeclareLaunchArgument(
        "crab_follower_yaw_integrator_limit",
        default_value="1.2",
        description="Integrator state limit for the crab follower yaw hold controller.",
    )
    crab_follower_yaw_hold_target_mode_arg = DeclareLaunchArgument(
        "crab_follower_yaw_hold_target_mode",
        default_value="path_heading",
        description="Use initial yaw hold or path_heading vector yaw hold.",
    )

    use_slam = LaunchConfiguration("use_slam")
    use_locomotion = LaunchConfiguration("use_locomotion")
    use_explorer = LaunchConfiguration("use_explorer")
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    raw_cmd_vel_topic = LaunchConfiguration("raw_cmd_vel_topic")
    safe_cmd_vel_topic = LaunchConfiguration("safe_cmd_vel_topic")
    cmd_vel_yaw_offset_rad = LaunchConfiguration("cmd_vel_yaw_offset_rad")
    crab_follower_speed_mps = LaunchConfiguration("crab_follower_speed_mps")
    crab_follower_goal_tolerance_m = LaunchConfiguration("crab_follower_goal_tolerance_m")
    crab_follower_yaw_ki = LaunchConfiguration("crab_follower_yaw_ki")
    crab_follower_yaw_integrator_limit = LaunchConfiguration(
        "crab_follower_yaw_integrator_limit"
    )
    crab_follower_yaw_hold_target_mode = LaunchConfiguration(
        "crab_follower_yaw_hold_target_mode"
    )

    # -----------------------------------------------------------------------
    # Config paths
    # -----------------------------------------------------------------------
    builder_params = PathJoinSubstitution([pkg_share, "config", "maze_graph_builder.yaml"])
    planner_params = PathJoinSubstitution([pkg_share, "config", "maze_graph_planner.yaml"])
    visualizer_params = PathJoinSubstitution([pkg_share, "config", "graph_visualizer.yaml"])
    slam_params = PathJoinSubstitution([pkg_share, "config", "slam_toolbox.yaml"])

    # -----------------------------------------------------------------------
    # slam_toolbox (optional)
    # -----------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ])
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(use_slam),
    )

    # -----------------------------------------------------------------------
    # maze_graph_builder
    # -----------------------------------------------------------------------
    builder_node = Node(
        package="hexapod_slam",
        executable="maze_graph_builder",
        name="maze_graph_builder",
        parameters=[builder_params, {"use_sim_time": use_sim_time, "odom_topic": odom_topic}],
        output="screen",
    )

    # -----------------------------------------------------------------------
    # maze_graph_planner
    # -----------------------------------------------------------------------
    planner_node = Node(
        package="hexapod_slam",
        executable="maze_graph_planner",
        name="maze_graph_planner",
        parameters=[planner_params, {"use_sim_time": use_sim_time, "odom_topic": odom_topic}],
        output="screen",
    )

    # -----------------------------------------------------------------------
    # maze_graph_visualizer
    # -----------------------------------------------------------------------
    visualizer_node = Node(
        package="hexapod_slam",
        executable="maze_graph_visualizer",
        name="maze_graph_visualizer",
        parameters=[visualizer_params, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # -----------------------------------------------------------------------
    # path_to_local_goal
    # -----------------------------------------------------------------------
    local_goal_node = Node(
        package="hexapod_slam",
        executable="path_to_local_goal",
        name="path_to_local_goal",
        parameters=[{"use_sim_time": use_sim_time, "odom_topic": odom_topic}],
        output="screen",
    )

    path_follower_node = Node(
        package="hexapod_locomotion",
        executable="crab_path_follower",
        name="crab_path_follower",
        parameters=[{
            "path_topic": "/maze_graph/path",
            "odom_topic": odom_topic,
            "cmd_vel_topic": raw_cmd_vel_topic,
            "constant_speed_mps": crab_follower_speed_mps,
            "goal_tolerance_m": crab_follower_goal_tolerance_m,
            "path_timeout_sec": 1.0,
            "cmd_vel_rate_hz": 20.0,
            "yaw_correction_gain": 0.6,
            "yaw_ki": crab_follower_yaw_ki,
            "yaw_integrator_limit": crab_follower_yaw_integrator_limit,
            "max_angular_speed_rad_s": 0.12,
            "yaw_deadband_deg": 5.0,
            "yaw_hold_target_mode": crab_follower_yaw_hold_target_mode,
            "cmd_vel_yaw_offset_rad": cmd_vel_yaw_offset_rad,
        }],
        output="screen",
        condition=IfCondition(use_locomotion),
    )

    safety_node = Node(
        package="hexapod_slam",
        executable="scan_cmd_vel_safety",
        name="scan_cmd_vel_safety",
        parameters=[{
            "scan_topic": scan_topic,
            "input_cmd_vel_topic": raw_cmd_vel_topic,
            "output_cmd_vel_topic": safe_cmd_vel_topic,
            "cmd_vel_yaw_offset_rad": cmd_vel_yaw_offset_rad,
            "control_rate_hz": 20.0,
            "scan_timeout_sec": 0.5,
            "cmd_timeout_sec": 0.5,
            "clearance_window_deg": 15.0,
            "stop_distance_m": 0.42,
            "slowdown_distance_m": 0.75,
            "side_clearance_window_deg": 50.0,
            "side_stop_distance_m": 0.38,
            "side_slowdown_distance_m": 0.60,
            "max_side_push_ratio": 0.55,
            "preserve_turning_when_blocked": False,
        }],
        output="screen",
        condition=IfCondition(use_locomotion),
    )

    # -----------------------------------------------------------------------
    # gap_following_explorer (optional)
    # -----------------------------------------------------------------------
    explorer_node = Node(
        package="hexapod_slam",
        executable="gap_following_explorer",
        name="gap_following_explorer",
        parameters=[{
            "use_sim_time": use_sim_time,
            # Stop further from walls to avoid collisions
            "stop_distance_m": 0.65,
            # More lenient gap threshold so robot finds a path in tight corridors
            "open_distance_m": 0.80,
            # Keep rolling goal further from the detected obstacle
            "goal_backoff_m": 0.45,
            # Wider clearance window catches walls approached at an angle
            "clearance_window_deg": 15.0,
            # Bias the robot to move forward on startup instead of picking a random gap
            "forward_bias_weight": 1.5,
        }],
        output="screen",
        condition=IfCondition(use_explorer),
    )

    return LaunchDescription([
        use_slam_arg,
        use_locomotion_arg,
        use_explorer_arg,
        use_sim_time_arg,
        odom_topic_arg,
        scan_topic_arg,
        raw_cmd_vel_topic_arg,
        safe_cmd_vel_topic_arg,
        cmd_vel_yaw_offset_arg,
        crab_follower_speed_arg,
        crab_follower_goal_tolerance_arg,
        crab_follower_yaw_ki_arg,
        crab_follower_yaw_integrator_limit_arg,
        crab_follower_yaw_hold_target_mode_arg,
        slam_launch,
        builder_node,
        planner_node,
        visualizer_node,
        local_goal_node,
        path_follower_node,
        safety_node,
        explorer_node,
    ])
