"""
graph_io.py

High-level purpose:
    Utility module for saving and loading the persistent maze graph to and from disk.
    This allows the graph built during exploration to be reused across runs and tied
    to a saved occupancy map.

High-level behavior:
    - Serializes graph nodes, edges, metadata, and optional planner state.
    - Deserializes a saved graph file back into in-memory graph objects.
    - Stores frame names, map file association, timestamps, and exploration status.

Inputs:
    - In-memory graph state
    - File path for save/load
    - Optional metadata such as associated map name

Outputs:
    - YAML or JSON graph files
    - Restored graph state objects

Does not publish or subscribe:
    - This is normally a helper module, not a standalone ROS node.

Suggested stored metadata:
    - map_frame
    - base_frame
    - associated_map_yaml
    - timestamp
    - node list
    - edge list
    - node types
    - edge states
    - visit counts
    - blocked edge markers
    - planner strategy metadata

Suggested default file paths:
    - graph_save_path: "maps/maze_graph.yaml"
    - graph_autosave_period_sec: 30.0

Notes:
    - Keep serialization logic separate from planning and graph extraction logic.
    - Use stable node IDs so saved graphs can be compared between runs.
"""