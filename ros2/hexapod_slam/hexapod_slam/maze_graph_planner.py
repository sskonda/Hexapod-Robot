"""
maze_graph_planner.py

High-level purpose:
    High-level maze exploration planner that decides which graph node or graph edge
    the robot should travel to next. This node uses the persistent topological graph
    built from the SLAM map and chooses unexplored branches, backtracking when needed.

High-level behavior:
    - Receives graph state from the graph builder.
    - Estimates the robot's current location relative to graph nodes.
    - Chooses the next target using a supported graph-search strategy.
    - Publishes the current target node / target pose and planned graph path.
    - Replans when a target is reached, blocked, or invalidated.

Supported planning strategies:
    - DFS ("dfs"):
        Depth-first exploration. Follows one branch as far as possible before
        backtracking. This is simple, memory-efficient, and a good default for
        complete maze exploration.
    - BFS ("bfs"):
        Breadth-first exploration. Expands nearby unexplored branches first.
        This is useful when you want more even coverage and shorter graph-depth
        routes to newly discovered nodes.

Inputs:
    - Persistent graph state from the graph builder
    - /odom (nav_msgs/msg/Odometry)
    - Optional local follower status: reached, blocked, stuck

Outputs:
    - Active target pose
    - Planned graph path
    - Planner debug state

Publishes:
    - /maze_graph/current_target -> geometry_msgs/msg/PoseStamped
    - /maze_graph/path -> nav_msgs/msg/Path
    - /maze_graph/planner_debug -> visualization_msgs/msg/MarkerArray or std_msgs/msg/String
    - /maze_graph/active_edge -> visualization_msgs/msg/MarkerArray (optional)

Subscribes:
    - /maze_graph/nodes or internal graph state source
    - /maze_graph/edges or internal graph state source
    - odom_topic (default: "/odom") -> nav_msgs/msg/Odometry
    - /maze_graph/local_status (optional)

Suggested default parameters:
    Topics and frames:
    - odom_topic: "/odom"
    - map_frame: "map"
    - base_frame: "base_link"

    Planning behavior:
    - planner_rate_hz: 2.0
    - strategy: "dfs"
        Supported values:
            * "dfs" = depth-first search
            * "bfs" = breadth-first search
    - target_reached_distance_m: 0.15
        Distance from target node used to declare success.
    - current_node_snap_distance_m: 0.20
        Maximum distance to associate the robot with a graph node.
    - blocked_edge_retry_limit: 2
        Number of failed attempts before marking an edge as blocked.
    - blocked_edge_cooldown_sec: 10.0
        Time before reconsidering a blocked edge.

    Optional scoring / tie-break gains:
    - revisit_penalty_gain: 1.0
        Penalizes repeatedly traversed edges.
    - unexplored_bonus_gain: 5.0
        Strong reward for edges leading into unexplored territory.
    - heading_alignment_gain: 0.5
        Optional preference for targets aligned with current robot heading.
    - backtrack_penalty_gain: 0.5
        Keeps the robot from backtracking too early when alternatives exist.

Notes:
    - This node should make long-horizon decisions only.
    - It should not directly command joint motion or gait behavior.
    - It is best paired with a local goal follower or obstacle-aware explorer.
    - DFS is typically the simplest default for full maze traversal.
    - BFS should also be supported for cases where shallower graph expansion is preferred.
"""