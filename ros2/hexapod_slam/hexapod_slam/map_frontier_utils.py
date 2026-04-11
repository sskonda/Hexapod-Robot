"""
map_frontier_utils.py

High-level purpose:
    Shared utility functions for processing occupancy grids and extracting traversability
    information used by the maze graph builder. This file contains reusable helper logic
    for map thresholding, inflation, frontier detection, and coordinate conversion.

High-level behavior:
    - Converts occupancy grid data into array form.
    - Classifies cells as free, occupied, or unknown.
    - Inflates obstacles for safe path planning.
    - Detects frontiers where free space meets unknown space.
    - Provides world <-> grid coordinate transforms.
    - Supports skeleton / neighbor / connectivity calculations.

Inputs:
    - nav_msgs/msg/OccupancyGrid data and metadata
    - Grid indices, world coordinates, and threshold values

Outputs:
    - Numpy arrays or helper results used by higher-level nodes

Does not publish or subscribe:
    - This is a pure helper module, not a standalone ROS node.

Typical helper functions:
    - occupancy_grid_to_array()
    - world_to_grid()
    - grid_to_world()
    - inflate_obstacles()
    - detect_frontiers()
    - classify_cell()
    - get_neighbor_indices()
    - line_of_sight_is_free()
    - heading_between_points()

Suggested default thresholds:
    - occupied_threshold: 50
    - free_threshold: 20
    - inflation_radius_cells: derived from inflation_radius_m / map resolution

Notes:
    - Keep map math and array utilities here so builder/planner files stay readable.
    - This file should not own persistent graph state.
"""