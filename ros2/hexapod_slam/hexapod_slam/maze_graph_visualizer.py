"""
graph_visualizer.py

High-level purpose:
    Dedicated RViz visualization node for the persistent maze graph, active planner state,
    and debug overlays. This node separates visualization concerns from graph building
    and planning logic.

High-level behavior:
    - Subscribes to graph state, target path, and robot state.
    - Publishes MarkerArray messages for nodes, edges, labels, frontiers, and current target.
    - Provides an easy RViz view of the persistent topological map overlaid on the SLAM map.

Inputs:
    - Maze graph node state
    - Maze graph edge state
    - Current planner target
    - Current planned path
    - /odom (optional)
    - /scan (optional for overlay)
    - /map (optional for context)

Outputs:
    - MarkerArray overlays for RViz

Publishes:
    - /maze_graph/markers -> visualization_msgs/msg/MarkerArray
    - /maze_graph/labels -> visualization_msgs/msg/MarkerArray
    - /maze_graph/frontier_markers -> visualization_msgs/msg/MarkerArray
    - /maze_graph/active_target_marker -> visualization_msgs/msg/MarkerArray
    - /maze_graph/traversed_path_marker -> visualization_msgs/msg/MarkerArray (optional)

Subscribes:
    - /maze_graph/nodes
    - /maze_graph/edges
    - /maze_graph/frontiers
    - /maze_graph/current_target
    - /maze_graph/path
    - /odom (optional)
    - /scan (optional)

Suggested default parameters:
    - marker_frame: "map"
    - publish_rate_hz: 2.0
    - node_scale_m: 0.08
    - edge_width_m: 0.02
    - frontier_scale_m: 0.05
    - label_text_scale_m: 0.10
    - show_node_labels: True
    - show_edge_labels: False
    - show_frontiers: True
    - show_dead_ends: True
    - show_active_target: True
    - show_debug_candidates: False

Recommended color conventions:
    - unexplored/frontier node: yellow
    - visited node: green
    - dead end: red
    - junction: blue
    - corner: cyan
    - active target: magenta
    - blocked edge: orange or dark red

Notes:
    - This node does not make decisions or modify graph state.
    - It only converts graph/planner state into RViz-friendly markers.
"""