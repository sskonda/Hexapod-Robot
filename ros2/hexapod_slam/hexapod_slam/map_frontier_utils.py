"""
map_frontier_utils.py — Occupancy grid utilities for maze graph extraction.
No scipy or scikit-image dependencies — pure numpy + stdlib.
"""

import math
from collections import deque
from typing import List, Tuple

import numpy as np


def occupancy_grid_to_array(msg) -> np.ndarray:
    """
    Reshape msg.data (flat int8 list) into a (height, width) int16 array.

    ROS OccupancyGrid stores -1 for unknown, 0-100 for probability.
    In uint8 the -1 wraps to 255.  We keep -1 as -1, 0-100 as-is.
    Returns int16 array.
    """
    height = msg.info.height
    width = msg.info.width
    raw = np.array(msg.data, dtype=np.int8).reshape((height, width))
    # Cast to int16 so -1 stays -1 (int8 -1 → int16 -1)
    return raw.astype(np.int16)


def world_to_grid(x_m: float, y_m: float,
                  origin_x: float, origin_y: float,
                  resolution: float) -> Tuple[int, int]:
    """Return (gx, gy) integers for the cell containing (x_m, y_m)."""
    gx = int(math.floor((x_m - origin_x) / resolution))
    gy = int(math.floor((y_m - origin_y) / resolution))
    return gx, gy


def grid_to_world(gx: int, gy: int,
                  origin_x: float, origin_y: float,
                  resolution: float) -> Tuple[float, float]:
    """Return (x_m, y_m) world coordinates at the centre of grid cell (gx, gy)."""
    x_m = origin_x + (gx + 0.5) * resolution
    y_m = origin_y + (gy + 0.5) * resolution
    return x_m, y_m


def make_binary_free(grid: np.ndarray, free_threshold: int = 20) -> np.ndarray:
    """
    Return a boolean array: True where the cell is known-free.

    Free  : grid >= 0  AND  grid <= free_threshold
    Not free: occupied (> free_threshold) or unknown (-1)
    """
    return (grid >= 0) & (grid <= free_threshold)


def inflate_obstacles(binary_free: np.ndarray, radius_cells: int) -> np.ndarray:
    """
    Erode free space by radius_cells (equivalent to inflating obstacles).

    Uses BFS seeded from every obstacle/unknown cell.  O(h*w) time.
    Returns a new bool array.
    """
    h, w = binary_free.shape
    result = binary_free.copy()
    if radius_cells <= 0:
        return result

    visited = ~binary_free.copy()  # True where already non-free
    queue: deque = deque()

    # Seed queue with every obstacle/unknown cell
    obstacle_ys, obstacle_xs = np.where(~binary_free)
    for gy, gx in zip(obstacle_ys.tolist(), obstacle_xs.tolist()):
        queue.append((gy, gx, 0))

    neighbours = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    while queue:
        gy, gx, dist = queue.popleft()
        if dist >= radius_cells:
            continue
        for dy, dx in neighbours:
            ny, nx = gy + dy, gx + dx
            if 0 <= ny < h and 0 <= nx < w and not visited[ny, nx]:
                visited[ny, nx] = True
                result[ny, nx] = False
                queue.append((ny, nx, dist + 1))

    return result


def detect_frontiers(grid: np.ndarray,
                     free_threshold: int = 20) -> List[Tuple[int, int]]:
    """
    Find frontier cells: free AND adjacent (4-connected) to at least one unknown cell.

    Returns list of (gx, gy) tuples.
    """
    h, w = grid.shape
    free_mask = make_binary_free(grid, free_threshold)
    unknown_mask = (grid == -1)

    # Shift unknown mask in four directions and OR together
    unknown_up = np.zeros_like(unknown_mask)
    unknown_down = np.zeros_like(unknown_mask)
    unknown_left = np.zeros_like(unknown_mask)
    unknown_right = np.zeros_like(unknown_mask)

    unknown_up[1:, :] = unknown_mask[:-1, :]
    unknown_down[:-1, :] = unknown_mask[1:, :]
    unknown_left[:, 1:] = unknown_mask[:, :-1]
    unknown_right[:, :-1] = unknown_mask[:, 1:]

    adjacent_unknown = unknown_up | unknown_down | unknown_left | unknown_right
    frontier_mask = free_mask & adjacent_unknown

    ys, xs = np.where(frontier_mask)
    return list(zip(xs.tolist(), ys.tolist()))  # (gx, gy)


def ray_cast_distance(binary_free: np.ndarray,
                      gx: int, gy: int,
                      angle_rad: float,
                      max_dist_cells: float) -> float:
    """
    Cast a ray from (gx, gy) in direction angle_rad.
    Returns distance in cells until hitting a non-free cell, or max_dist_cells.
    Steps in 0.5-cell increments.
    """
    h, w = binary_free.shape
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    step = 0.5
    t = step
    while t <= max_dist_cells:
        nx = gx + cos_a * t
        ny = gy + sin_a * t
        igx = int(round(nx))
        igy = int(round(ny))
        if not (0 <= igx < w and 0 <= igy < h):
            return t - step
        if not binary_free[igy, igx]:
            return t - step
        t += step
    return max_dist_cells


def heading_between_points(x1: float, y1: float,
                           x2: float, y2: float) -> float:
    """Return angle in radians from (x1,y1) to (x2,y2)."""
    return math.atan2(y2 - y1, x2 - x1)


def distance_m(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance between two 2-D points."""
    return math.hypot(x2 - x1, y2 - y1)


def line_of_sight_is_free(binary_free: np.ndarray,
                           gx1: int, gy1: int,
                           gx2: int, gy2: int) -> bool:
    """
    Bresenham line check: True if every grid cell on the line from
    (gx1,gy1) to (gx2,gy2) is free.
    """
    h, w = binary_free.shape
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    x, y = gx1, gy1
    sx = 1 if gx2 > gx1 else -1
    sy = 1 if gy2 > gy1 else -1

    if dx > dy:
        err = dx // 2
        while x != gx2:
            if not (0 <= x < w and 0 <= y < h):
                return False
            if not binary_free[y, x]:
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while y != gy2:
            if not (0 <= x < w and 0 <= y < h):
                return False
            if not binary_free[y, x]:
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    # Check final cell
    if not (0 <= gx2 < w and 0 <= gy2 < h):
        return False
    return bool(binary_free[gy2, gx2])
