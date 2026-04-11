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

import os

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
        description="Launch the locomotion / gap-following explorer.",
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

    use_slam = LaunchConfiguration("use_slam")
    use_locomotion = LaunchConfiguration("use_locomotion")
    use_explorer = LaunchConfiguration("use_explorer")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
        parameters=[builder_params, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # -----------------------------------------------------------------------
    # maze_graph_planner
    # -----------------------------------------------------------------------
    planner_node = Node(
        package="hexapod_slam",
        executable="maze_graph_planner",
        name="maze_graph_planner",
        parameters=[planner_params, {"use_sim_time": use_sim_time}],
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
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # -----------------------------------------------------------------------
    # gap_following_explorer (optional)
    # -----------------------------------------------------------------------
    explorer_node = Node(
        package="hexapod_slam",
        executable="gap_following_explorer",
        name="gap_following_explorer",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_explorer),
    )

    return LaunchDescription([
        use_slam_arg,
        use_locomotion_arg,
        use_explorer_arg,
        use_sim_time_arg,
        slam_launch,
        builder_node,
        planner_node,
        visualizer_node,
        local_goal_node,
        explorer_node,
    ])
