"""
path_to_local_goal.py

High-level purpose:
    Converts a high-level graph target or graph path into a short-horizon local goal
    that the low-level local navigation node can follow safely. This node bridges the
    gap between graph planning and reactive obstacle-aware movement.

High-level behavior:
    - Receives the current graph target or route from the high-level planner.
    - Chooses a near-term waypoint ahead of the robot.
    - Optionally adjusts that waypoint using current LiDAR or local map information.
    - Publishes a local goal pose or short local path for the local follower.

Inputs:
    - /maze_graph/current_target (geometry_msgs/msg/PoseStamped)
    - /maze_graph/path (nav_msgs/msg/Path)
    - /odom (nav_msgs/msg/Odometry)
    - /scan (sensor_msgs/msg/LaserScan), optional
    - /map (nav_msgs/msg/OccupancyGrid), optional

Outputs:
    - Local goal pose
    - Short local path
    - Goal status / progress feedback

Publishes:
    - /maze_graph/local_goal -> geometry_msgs/msg/PoseStamped
    - /maze_graph/local_path -> nav_msgs/msg/Path
    - /maze_graph/local_status -> std_msgs/msg/String or custom status topic

Subscribes:
    - /maze_graph/current_target
    - /maze_graph/path
    - odom_topic (default: "/odom")
    - scan_topic (default: "/scan"), optional

Suggested default parameters:
    - update_rate_hz: 5.0
    - lookahead_distance_m: 0.25
    - goal_tolerance_m: 0.10
    - path_resample_spacing_m: 0.05
    - obstacle_clearance_m: 0.10
    - target_timeout_sec: 2.0
    - use_scan_refinement: True

Optional gain-like parameters:
    - heading_alignment_gain: 1.0
        Helps pick local goals aligned with the target edge direction.
    - obstacle_avoidance_gain: 1.0
        Pushes the selected local point away from nearby obstacles.
    - path_smoothing_gain: 0.5
        Reduces rapid goal jitter.

Notes:
    - This node should not do high-level exploration decisions.
    - It should only translate graph intent into short local navigation goals.
"""