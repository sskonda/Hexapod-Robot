"""
graph_types.py

High-level purpose:
    Defines the shared data structures used by the maze graph system. This includes
    node types, edge types, graph state containers, and helper enums for exploration status.

High-level behavior:
    - Provides consistent in-memory representations for persistent graph objects.
    - Prevents duplication of node/edge definitions across builder, planner, and visualizer code.
    - Centralizes shared constants and state labels.

Inputs:
    - Internal use only by other modules

Outputs:
    - Python dataclasses / enums / helper containers

Does not publish or subscribe:
    - This is a shared definitions module, not a ROS node.

Suggested structures:
    - MazeNode
        Fields:
            id
            x_m
            y_m
            yaw_rad (optional)
            node_type
            visit_state
            confidence
            observation_count
            last_seen_time
            connected_edge_ids

    - MazeEdge
        Fields:
            id
            start_node_id
            end_node_id
            polyline_points
            length_m
            heading_rad
            edge_state
            visit_count
            blocked_count
            confidence

    - GraphState
        Fields:
            nodes
            edges
            frontier_edges
            active_target_node_id
            version
            timestamp

Suggested enums:
    - NodeType:
        START
        CORNER
        JUNCTION
        DEAD_END
        FRONTIER
        GOAL_CANDIDATE

    - VisitState:
        UNSEEN
        SEEN
        VISITED
        BLOCKED

    - EdgeState:
        UNEXPLORED
        OPEN
        VISITED
        BLOCKED

Notes:
    - Keep these structures lightweight and easy to serialize.
    - Use map-frame coordinates for persistent positions.
"""