"""
maze_mapping.launch.py

Purpose:
    Main launch file for persistent maze mapping and topological exploration.

High-level behavior:
    - Launches slam_toolbox for occupancy-grid SLAM.
    - Launches maze_graph_builder to extract persistent decision nodes and edges.
    - Launches maze_graph_planner to choose the next branch to explore.
    - Launches graph_visualizer for RViz markers.
    - Optionally launches the local follower / local explorer stack.

Inputs:
    - Launch arguments for topic names, frame names, and parameter files.

Outputs:
    - Complete maze exploration system running under one launch command.

Typical launched nodes:
    - slam_toolbox
    - maze_graph_builder
    - maze_graph_planner
    - graph_visualizer
    - path_to_local_goal
    - gap_following_explorer or local path follower
"""