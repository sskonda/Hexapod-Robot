"""
maze_graph_builder.py

High-level purpose:
    Builds and maintains a persistent topological maze graph from the SLAM occupancy map,
    LiDAR observations, and robot pose. This node identifies meaningful decision points
    such as corners, junctions, and dead ends, and stores them as persistent graph nodes
    in the map frame.

High-level behavior:
    - Subscribes to the persistent occupancy grid from SLAM.
    - Optionally uses current LiDAR data to validate nearby free space and frontiers.
    - Detects candidate graph features such as:
        * corners
        * T-junctions
        * intersections
        * dead ends
        * unexplored frontiers
    - Merges repeated detections into stable persistent nodes.
    - Connects nodes with edges along traversable free-space structure.
    - Publishes graph state and debug markers for RViz.

Inputs:
    - /map (nav_msgs/msg/OccupancyGrid)
    - /scan (sensor_msgs/msg/LaserScan), optional but recommended
    - /odom (nav_msgs/msg/Odometry)
    - TF map -> base_link

Outputs:
    - Persistent node and edge state
    - Frontier candidates
    - RViz debug markers

Publishes:
    - /maze_graph/nodes -> visualization_msgs/msg/MarkerArray
    - /maze_graph/edges -> visualization_msgs/msg/MarkerArray
    - /maze_graph/frontiers -> visualization_msgs/msg/MarkerArray
    - /maze_graph/debug/candidates -> visualization_msgs/msg/MarkerArray
    - /maze_graph/debug/skeleton -> visualization_msgs/msg/MarkerArray (optional)

Subscribes:
    - map_topic (default: "/map") -> nav_msgs/msg/OccupancyGrid
    - scan_topic (default: "/scan") -> sensor_msgs/msg/LaserScan
    - odom_topic (default: "/odom") -> nav_msgs/msg/Odometry

Suggested default parameters:
    Topics and frames:
    - map_topic: "/map"
    - scan_topic: "/scan"
    - odom_topic: "/odom"
    - map_frame: "map"
    - base_frame: "base_link"

    Update behavior:
    - update_rate_hz: 2.0
        Frequency for rebuilding / updating graph features.
    - min_feature_observations: 3
        Number of repeated detections before a feature becomes a persistent node.
    - node_merge_distance_m: 0.20
        Maximum distance between detections to treat them as the same node.
    - min_node_spacing_m: 0.30
        Prevents placing many nodes too close together.

    Map processing:
    - occupied_threshold: 50
        Occupancy values above this are treated as obstacles.
    - free_threshold: 20
        Occupancy values below this are treated as free space.
    - inflation_radius_m: 0.10
        Obstacle inflation radius for safe traversability checks.
    - robot_clearance_radius_m: 0.12
        Approximate robot half-width plus safety margin.
    - unknown_is_blocked: False
        Whether unknown cells should be treated as blocked for edge generation.

    Feature detection:
    - corner_angle_threshold_deg: 35.0
        Minimum heading change to classify a degree-2 skeleton point as a corner.
    - dead_end_min_length_m: 0.25
        Minimum corridor length to keep a dead-end node.
    - junction_min_branch_separation_deg: 25.0
        Minimum angular separation between outgoing branches.
    - frontier_min_length_m: 0.15
        Minimum frontier extent to publish as an unexplored edge candidate.

Notes:
    - This node should operate in the map frame to reduce duplication from odometry drift.
    - The graph is intended to be persistent across the exploration session.
    - A later save/load utility can serialize this graph to disk.
"""