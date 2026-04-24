#!/usr/bin/env python3
"""
maze_graph_planner.py

DFS/BFS exploration planner that reads /maze_graph/graph_json and publishes
the next navigation target and planned path.
"""

import math
import time
from collections import deque
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .frame_utils import lookup_point_2d
from .graph_io import json_to_graph
from .graph_types import CellSideState, EdgeState, GraphState, MazeCell, MazeNode, NodeType, VisitState
from .map_frontier_utils import (
    distance_m,
    heading_between_points,
    inflate_obstacles,
    line_of_sight_is_free,
    make_binary_free,
    occupancy_grid_to_array,
    ray_cast_distance,
    world_to_grid,
)
from .maze_cell_memory import classify_cell_side


# ---------------------------------------------------------------------------
# Pure planning helpers (testable without ROS2)
# ---------------------------------------------------------------------------

def find_closest_node(
    graph: GraphState,
    x: float,
    y: float,
    snap_distance_m: float,
) -> Optional[int]:
    """Return the id of the nearest node within snap_distance_m, or None."""
    best_id: Optional[int] = None
    best_dist = snap_distance_m
    for nid, node in graph.nodes.items():
        d = distance_m(x, y, node.x_m, node.y_m)
        if d < best_dist:
            best_dist = d
            best_id = nid
    return best_id


def select_next_target_dfs(
    graph: GraphState,
    current_node_id: int,
    blocked_edges: Set[int],
    unexplored_bonus: float = 5.0,
    revisit_penalty: float = 1.0,
    cell_commit_confidence: float = 0.65,
    cell_dominance_margin: float = 0.15,
    cell_unvisited_bonus_gain: float = 1.5,
    cell_revisit_penalty_gain: float = 0.35,
    cell_opening_bonus_gain: float = 0.25,
) -> Optional[int]:
    """
    DFS-style selection: prefer unexplored edges, penalise revisited nodes.
    Returns the id of the best next node, or None if all are exhausted.
    """
    best_id: Optional[int] = None
    best_score = -float("inf")

    current = graph.nodes.get(current_node_id)
    if current is None:
        return None

    for edge in graph.get_node_edges(current_node_id):
        if edge.id in blocked_edges:
            continue
        if edge.edge_state == EdgeState.BLOCKED:
            continue

        other_id = edge.end_node_id if edge.start_node_id == current_node_id else edge.start_node_id
        other = graph.nodes.get(other_id)
        if other is None:
            continue

        score = 0.0
        if edge.edge_state == EdgeState.UNEXPLORED:
            score += unexplored_bonus
        if other.visit_state == VisitState.VISITED:
            score -= revisit_penalty * edge.visit_count
        if other.node_type == NodeType.FRONTIER:
            score += unexplored_bonus * 0.5
        score += node_cell_score(
            graph,
            other_id,
            cell_commit_confidence,
            cell_dominance_margin,
            cell_unvisited_bonus_gain,
            cell_revisit_penalty_gain,
            cell_opening_bonus_gain,
        )

        if score > best_score:
            best_score = score
            best_id = other_id

    return best_id


def select_next_target_bfs(
    graph: GraphState,
    current_node_id: int,
    blocked_edges: Set[int],
) -> Optional[int]:
    """
    BFS from current_node_id; return the nearest unvisited/unexplored node.
    """
    visited_bfs: Set[int] = {current_node_id}
    queue: deque = deque([current_node_id])

    while queue:
        node_id = queue.popleft()
        node = graph.nodes.get(node_id)
        if node is None:
            continue

        for edge in graph.get_node_edges(node_id):
            if edge.id in blocked_edges or edge.edge_state == EdgeState.BLOCKED:
                continue
            other_id = (edge.end_node_id
                        if edge.start_node_id == node_id
                        else edge.start_node_id)
            if other_id in visited_bfs:
                continue
            visited_bfs.add(other_id)
            other = graph.nodes.get(other_id)
            if other is None:
                continue
            if other.visit_state != VisitState.VISITED:
                return other_id
            queue.append(other_id)

    return None


def build_path_to_node(
    graph: GraphState,
    start_id: int,
    target_id: int,
    blocked_edges: Set[int],
) -> List[int]:
    """BFS to find a list of node ids from start_id to target_id."""
    if start_id == target_id:
        return [start_id]

    prev: Dict[int, Optional[int]] = {start_id: None}
    queue: deque = deque([start_id])

    while queue:
        nid = queue.popleft()
        if nid == target_id:
            break
        for edge in graph.get_node_edges(nid):
            if edge.id in blocked_edges or edge.edge_state == EdgeState.BLOCKED:
                continue
            other_id = (edge.end_node_id
                        if edge.start_node_id == nid
                        else edge.start_node_id)
            if other_id not in prev:
                prev[other_id] = nid
                queue.append(other_id)

    if target_id not in prev:
        return [start_id]  # no path found

    path: List[int] = []
    cur: Optional[int] = target_id
    while cur is not None:
        path.append(cur)
        cur = prev.get(cur)
    path.reverse()
    return path


def find_edge_between_nodes(
    graph: GraphState,
    start_id: int,
    end_id: int,
) -> Optional[int]:
    for edge in graph.get_node_edges(start_id):
        other_id = (
            edge.end_node_id
            if edge.start_node_id == start_id
            else edge.start_node_id
        )
        if other_id == end_id:
            return edge.id
    return None


def associated_cells_for_node(
    graph: GraphState,
    node_id: int,
) -> List[MazeCell]:
    return [
        cell
        for cell in graph.cells.values()
        if node_id in cell.associated_node_ids
    ]


def node_cell_score(
    graph: GraphState,
    node_id: int,
    commit_confidence: float,
    dominance_margin: float,
    unvisited_bonus_gain: float,
    revisit_penalty_gain: float,
    opening_bonus_gain: float,
) -> float:
    cells = associated_cells_for_node(graph, node_id)
    if not cells:
        return 0.0

    unvisited_fraction = sum(1 for cell in cells if cell.visit_count <= 0) / len(cells)
    min_visit_count = min(cell.visit_count for cell in cells)
    best_opening_count = max(
        sum(
            1
            for side_index in range(4)
            if classify_cell_side(
                cell,
                side_index,
                commit_confidence,
                dominance_margin,
            ) == CellSideState.OPEN
        )
        for cell in cells
    )

    return (
        unvisited_bonus_gain * unvisited_fraction
        - revisit_penalty_gain * float(min_visit_count)
        + opening_bonus_gain * float(best_opening_count)
    )


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _point_is_free(
    binary_free: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    x_m: float,
    y_m: float,
) -> Optional[Tuple[int, int]]:
    """Return the point's grid cell when it lies inside free space, else None."""
    if resolution <= 0.0 or binary_free.size == 0:
        return None

    gx, gy = world_to_grid(x_m, y_m, origin_x, origin_y, resolution)
    height, width = binary_free.shape
    if not (0 <= gx < width and 0 <= gy < height):
        return None
    if not binary_free[gy, gx]:
        return None
    return gx, gy


def _ray_clearance_m(
    binary_free: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    x_m: float,
    y_m: float,
    angle_rad: float,
    max_distance_m: float,
) -> Optional[float]:
    """Return free-space clearance from the world point along angle_rad."""
    cell = _point_is_free(binary_free, origin_x, origin_y, resolution, x_m, y_m)
    if cell is None or max_distance_m <= 0.0:
        return None

    gx, gy = cell
    max_distance_cells = max_distance_m / resolution
    return ray_cast_distance(binary_free, gx, gy, angle_rad, max_distance_cells) * resolution


def _bounded_side_walls(
    left_clearance_m: float,
    right_clearance_m: float,
    probe_distance_m: float,
    resolution: float,
) -> bool:
    """True when both side rays hit a wall within the probe horizon."""
    wall_detection_margin_m = max(0.5 * resolution, 1e-3)
    return (
        left_clearance_m < probe_distance_m - wall_detection_margin_m
        and right_clearance_m < probe_distance_m - wall_detection_margin_m
    )


def _sample_line_offsets(window_m: float, step_m: float) -> List[float]:
    """Return symmetric offsets in metres spanning [-window_m, window_m]."""
    if window_m <= 0.0 or step_m <= 0.0:
        return [0.0]

    step_count = max(1, int(math.ceil(window_m / step_m)))
    offsets: List[float] = [0.0]
    for step_index in range(1, step_count + 1):
        offset_m = min(window_m, step_index * step_m)
        if offset_m not in offsets:
            offsets.extend([offset_m, -offset_m])
    offsets.sort()
    return offsets


def _side_wall_profile(
    binary_free: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    x_m: float,
    y_m: float,
    heading_rad: float,
    probe_distance_m: float,
    longitudinal_window_m: float,
    longitudinal_step_m: float,
    required_bounded_fraction: float,
) -> Optional[Tuple[float, float, float, float]]:
    """Estimate corridor side-wall clearances around a point.

    Returns `(left_clearance_m, right_clearance_m, min_side_m, bounded_fraction)`
    when enough nearby longitudinal samples observe walls on both sides.
    """
    forward_x = math.cos(heading_rad)
    forward_y = math.sin(heading_rad)
    positive_side_angle = heading_rad + (math.pi / 2.0)
    negative_side_angle = heading_rad - (math.pi / 2.0)

    valid_samples = 0
    bounded_samples = 0
    bounded_left_values: List[float] = []
    bounded_right_values: List[float] = []

    for longitudinal_offset_m in _sample_line_offsets(
        longitudinal_window_m, longitudinal_step_m
    ):
        sample_x_m = x_m + forward_x * longitudinal_offset_m
        sample_y_m = y_m + forward_y * longitudinal_offset_m
        if _point_is_free(
            binary_free,
            origin_x,
            origin_y,
            resolution,
            sample_x_m,
            sample_y_m,
        ) is None:
            continue

        left_clearance_m = _ray_clearance_m(
            binary_free,
            origin_x,
            origin_y,
            resolution,
            sample_x_m,
            sample_y_m,
            positive_side_angle,
            probe_distance_m,
        )
        right_clearance_m = _ray_clearance_m(
            binary_free,
            origin_x,
            origin_y,
            resolution,
            sample_x_m,
            sample_y_m,
            negative_side_angle,
            probe_distance_m,
        )
        if left_clearance_m is None or right_clearance_m is None:
            continue

        valid_samples += 1
        if _bounded_side_walls(
            left_clearance_m, right_clearance_m, probe_distance_m, resolution
        ):
            bounded_samples += 1
            bounded_left_values.append(left_clearance_m)
            bounded_right_values.append(right_clearance_m)

    if valid_samples == 0 or bounded_samples == 0:
        return None

    bounded_fraction = bounded_samples / valid_samples
    if bounded_fraction < required_bounded_fraction:
        return None

    representative_left_m = float(np.median(bounded_left_values))
    representative_right_m = float(np.median(bounded_right_values))
    return (
        representative_left_m,
        representative_right_m,
        min(representative_left_m, representative_right_m),
        bounded_fraction,
    )


def regulate_target_between_walls(
    binary_free: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    nominal_x_m: float,
    nominal_y_m: float,
    heading_rad: float,
    max_lateral_offset_m: float,
    lateral_step_m: float,
    probe_distance_m: float,
    min_improvement_m: float = 0.03,
    longitudinal_window_m: float = 0.20,
    longitudinal_step_m: float = 0.05,
    required_bounded_fraction: float = 0.45,
) -> Tuple[float, float, bool]:
    """Shift a target laterally to maximize the minimum distance to side walls.

    The search is constrained to the cross-section perpendicular to heading_rad.
    If side walls are not detected within probe_distance_m the nominal target is
    returned unchanged so open junctions and rooms are left alone.
    """
    if (
        binary_free.size == 0
        or resolution <= 0.0
        or max_lateral_offset_m <= 0.0
        or lateral_step_m <= 0.0
        or probe_distance_m <= 0.0
    ):
        return nominal_x_m, nominal_y_m, False

    nominal_cell = _point_is_free(
        binary_free, origin_x, origin_y, resolution, nominal_x_m, nominal_y_m
    )
    if nominal_cell is None:
        return nominal_x_m, nominal_y_m, False

    lateral_x = -math.sin(heading_rad)
    lateral_y = math.cos(heading_rad)
    nominal_profile = _side_wall_profile(
        binary_free,
        origin_x,
        origin_y,
        resolution,
        nominal_x_m,
        nominal_y_m,
        heading_rad,
        probe_distance_m,
        longitudinal_window_m,
        longitudinal_step_m,
        required_bounded_fraction,
    )
    if nominal_profile is None:
        return nominal_x_m, nominal_y_m, False

    nominal_left, nominal_right, nominal_min_side_m, _ = nominal_profile

    step_count = max(1, int(math.ceil(max_lateral_offset_m / lateral_step_m)))
    offsets_m: List[float] = [0.0]
    for step_index in range(1, step_count + 1):
        offset_m = min(max_lateral_offset_m, step_index * lateral_step_m)
        if offset_m not in offsets_m:
            offsets_m.extend([offset_m, -offset_m])

    best_x_m = nominal_x_m
    best_y_m = nominal_y_m
    best_offset_m = 0.0
    best_min_side_m = nominal_min_side_m
    best_side_delta_m = abs(nominal_left - nominal_right)

    nominal_gx, nominal_gy = nominal_cell
    for offset_m in offsets_m:
        candidate_x_m = nominal_x_m + lateral_x * offset_m
        candidate_y_m = nominal_y_m + lateral_y * offset_m
        candidate_cell = _point_is_free(
            binary_free,
            origin_x,
            origin_y,
            resolution,
            candidate_x_m,
            candidate_y_m,
        )
        if candidate_cell is None:
            continue

        candidate_gx, candidate_gy = candidate_cell
        if offset_m != 0.0 and not line_of_sight_is_free(
            binary_free, nominal_gx, nominal_gy, candidate_gx, candidate_gy
        ):
            continue

        profile = _side_wall_profile(
            binary_free,
            origin_x,
            origin_y,
            resolution,
            candidate_x_m,
            candidate_y_m,
            heading_rad,
            probe_distance_m,
            longitudinal_window_m,
            longitudinal_step_m,
            required_bounded_fraction,
        )
        if profile is None:
            continue

        left_clearance_m, right_clearance_m, min_side_m, _ = profile
        side_delta_m = abs(left_clearance_m - right_clearance_m)
        if (
            min_side_m > best_min_side_m + 1e-6
            or (
                abs(min_side_m - best_min_side_m) <= 1e-6
                and side_delta_m < best_side_delta_m - 1e-6
            )
            or (
                abs(min_side_m - best_min_side_m) <= 1e-6
                and abs(side_delta_m - best_side_delta_m) <= 1e-6
                and abs(offset_m) < abs(best_offset_m) - 1e-6
            )
        ):
            best_x_m = candidate_x_m
            best_y_m = candidate_y_m
            best_offset_m = offset_m
            best_min_side_m = min_side_m
            best_side_delta_m = side_delta_m

    if abs(best_offset_m) < 1e-6:
        return nominal_x_m, nominal_y_m, False

    if best_min_side_m < nominal_min_side_m + min_improvement_m:
        return nominal_x_m, nominal_y_m, False

    return best_x_m, best_y_m, True


class MazeGraphPlanner(Node):
    def __init__(self):
        super().__init__("maze_graph_planner")

        self.declare_parameter("strategy", "dfs")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("planner_rate_hz", 2.0)
        self.declare_parameter("target_reached_distance_m", 0.20)
        self.declare_parameter("current_node_snap_distance_m", 0.25)
        self.declare_parameter("blocked_edge_retry_limit", 2)
        self.declare_parameter("blocked_edge_cooldown_sec", 10.0)
        self.declare_parameter("unexplored_bonus_gain", 5.0)
        self.declare_parameter("revisit_penalty_gain", 1.0)
        self.declare_parameter("heading_alignment_gain", 0.5)
        self.declare_parameter("backtrack_penalty_gain", 0.5)
        self.declare_parameter("free_threshold", 20)
        self.declare_parameter("cell_memory_enabled", True)
        self.declare_parameter("cell_commit_confidence", 0.65)
        self.declare_parameter("cell_dominance_margin", 0.15)
        self.declare_parameter("cell_unvisited_bonus_gain", 1.5)
        self.declare_parameter("cell_revisit_penalty_gain", 0.35)
        self.declare_parameter("cell_opening_bonus_gain", 0.25)
        self.declare_parameter("progress_stall_timeout_sec", 8.0)
        self.declare_parameter("progress_stall_min_improvement_m", 0.10)
        self.declare_parameter("wall_centering_enabled", True)
        self.declare_parameter("wall_centering_inflation_radius_m", 0.20)
        self.declare_parameter("wall_centering_max_lateral_offset_m", 0.30)
        self.declare_parameter("wall_centering_step_m", 0.05)
        self.declare_parameter("wall_centering_probe_distance_m", 0.80)
        self.declare_parameter("wall_centering_min_improvement_m", 0.03)
        self.declare_parameter("wall_centering_longitudinal_window_m", 0.20)
        self.declare_parameter("wall_centering_longitudinal_step_m", 0.05)
        self.declare_parameter("wall_centering_required_bounded_fraction", 0.45)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self._strategy = self.get_parameter("strategy").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._map_topic = self.get_parameter("map_topic").value
        self._rate = self.get_parameter("planner_rate_hz").value
        self._target_dist = self.get_parameter("target_reached_distance_m").value
        self._snap_dist = self.get_parameter("current_node_snap_distance_m").value
        self._retry_limit = self.get_parameter("blocked_edge_retry_limit").value
        self._cooldown = self.get_parameter("blocked_edge_cooldown_sec").value
        self._unexplored_bonus = self.get_parameter("unexplored_bonus_gain").value
        self._revisit_penalty = self.get_parameter("revisit_penalty_gain").value
        self._free_threshold = int(self.get_parameter("free_threshold").value)
        self._cell_memory_enabled = bool(self.get_parameter("cell_memory_enabled").value)
        self._cell_commit_confidence = float(
            self.get_parameter("cell_commit_confidence").value
        )
        self._cell_dominance_margin = float(
            self.get_parameter("cell_dominance_margin").value
        )
        self._cell_unvisited_bonus_gain = float(
            self.get_parameter("cell_unvisited_bonus_gain").value
        )
        self._cell_revisit_penalty_gain = float(
            self.get_parameter("cell_revisit_penalty_gain").value
        )
        self._cell_opening_bonus_gain = float(
            self.get_parameter("cell_opening_bonus_gain").value
        )
        self._progress_stall_timeout_sec = float(
            self.get_parameter("progress_stall_timeout_sec").value
        )
        self._progress_stall_min_improvement_m = float(
            self.get_parameter("progress_stall_min_improvement_m").value
        )
        self._wall_centering_enabled = bool(
            self.get_parameter("wall_centering_enabled").value
        )
        self._wall_centering_inflation_radius_m = float(
            self.get_parameter("wall_centering_inflation_radius_m").value
        )
        self._wall_centering_max_lateral_offset_m = float(
            self.get_parameter("wall_centering_max_lateral_offset_m").value
        )
        self._wall_centering_step_m = float(
            self.get_parameter("wall_centering_step_m").value
        )
        self._wall_centering_probe_distance_m = float(
            self.get_parameter("wall_centering_probe_distance_m").value
        )
        self._wall_centering_min_improvement_m = float(
            self.get_parameter("wall_centering_min_improvement_m").value
        )
        self._wall_centering_longitudinal_window_m = float(
            self.get_parameter("wall_centering_longitudinal_window_m").value
        )
        self._wall_centering_longitudinal_step_m = float(
            self.get_parameter("wall_centering_longitudinal_step_m").value
        )
        self._wall_centering_required_bounded_fraction = float(
            self.get_parameter("wall_centering_required_bounded_fraction").value
        )
        self._map_frame = self.get_parameter("map_frame").value

        # State
        self._graph: Optional[GraphState] = None
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_pose_frame: str = self._odom_topic
        self._target_node_id: Optional[int] = None
        self._current_path: List[int] = []
        self._blocked_edges: Set[int] = set()
        self._blocked_times: Dict[int, float] = {}
        self._safe_free_map: Optional[np.ndarray] = None
        self._map_origin_x: float = 0.0
        self._map_origin_y: float = 0.0
        self._map_resolution: float = 0.0
        self._last_tf_warn_time: float = 0.0
        self._last_progress_time: Optional[float] = None
        self._best_target_distance_m: Optional[float] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Subscriptions
        self.create_subscription(String, "/maze_graph/graph_json", self._graph_callback, 1)
        self.create_subscription(OccupancyGrid, self._map_topic, self._map_callback, 1)
        self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 10)

        # Publishers
        self._pub_target = self.create_publisher(PoseStamped, "/maze_graph/current_target", 1)
        self._pub_path = self.create_publisher(Path, "/maze_graph/path", 1)

        period = 1.0 / max(0.1, self._rate)
        self.create_timer(period, self._plan_timer)

        self.get_logger().info(f"MazeGraphPlanner started (strategy={self._strategy}).")

    # ------------------------------------------------------------------
    def _graph_callback(self, msg: String):
        try:
            self._graph = json_to_graph(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Graph parse error: {exc}")

    def _map_callback(self, msg: OccupancyGrid):
        resolution = msg.info.resolution
        if resolution <= 0.0:
            return

        try:
            grid = occupancy_grid_to_array(msg)
        except Exception as exc:
            self.get_logger().warn(f"Planner map parse error: {exc}")
            return

        binary_free = make_binary_free(grid, self._free_threshold)
        radius_cells = max(
            0, int(math.ceil(self._wall_centering_inflation_radius_m / resolution))
        )
        self._safe_free_map = inflate_obstacles(binary_free, radius_cells)
        self._map_origin_x = msg.info.origin.position.x
        self._map_origin_y = msg.info.origin.position.y
        self._map_resolution = resolution

    def _odom_callback(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        if msg.header.frame_id:
            self._robot_pose_frame = msg.header.frame_id

    # ------------------------------------------------------------------
    def _plan_timer(self):
        if self._graph is None or not self._graph.nodes:
            return

        # Expire blocked edges after cooldown
        now = time.time()
        expired = [eid for eid, t in self._blocked_times.items() if now - t > self._cooldown]
        for eid in expired:
            self._blocked_edges.discard(eid)
            del self._blocked_times[eid]

        self._plan()

    def _robot_position_in_map(self) -> Optional[Tuple[float, float]]:
        robot_map_position = lookup_point_2d(
            self._tf_buffer,
            self._map_frame,
            self._robot_pose_frame,
            self._robot_x,
            self._robot_y,
        )
        if robot_map_position is None and self._map_frame == self._robot_pose_frame:
            return self._robot_x, self._robot_y
        return robot_map_position

    def _plan(self):
        graph = self._graph
        robot_map_position = self._robot_position_in_map()
        if robot_map_position is None:
            now = time.time()
            if now - self._last_tf_warn_time > 5.0:
                self.get_logger().warn(
                    f'Unable to transform robot pose from {self._robot_pose_frame} '
                    f'to {self._map_frame}; planner update skipped.'
                )
                self._last_tf_warn_time = now
            return
        rx, ry = robot_map_position

        # 1. Find current node
        current_id = find_closest_node(graph, rx, ry, self._snap_dist)

        # 2. Check if target reached
        if self._target_node_id is not None:
            target_node = graph.nodes.get(self._target_node_id)
            if target_node is not None:
                target_x_m, target_y_m = self._regulated_target_position(
                    self._target_node_id,
                    current_id=current_id,
                    path_ids=self._current_path,
                )
                dist = distance_m(rx, ry, target_x_m, target_y_m)
                if dist < self._target_dist:
                    # Mark visited
                    target_node.visit_state = VisitState.VISITED
                    # Update edges leading to this node
                    for edge in graph.get_node_edges(self._target_node_id):
                        if edge.edge_state != EdgeState.BLOCKED:
                            edge.edge_state = EdgeState.VISITED
                            edge.visit_count += 1
                    self._target_node_id = None
                    self._current_path = []
                    self._last_progress_time = None
                    self._best_target_distance_m = None
                elif self._stalled_on_target(dist):
                    self._mark_current_path_blocked(
                        current_id,
                        (
                            f'progress toward node {self._target_node_id} stalled for '
                            f'{self._progress_stall_timeout_sec:.1f} s'
                        ),
                    )

        # 3. Select new target if needed
        if self._target_node_id is None:
            from_id = current_id
            if from_id is None:
                # Just pick the first unseen node
                for nid, node in graph.nodes.items():
                    if node.visit_state != VisitState.VISITED:
                        self._target_node_id = nid
                        break
            else:
                if self._strategy == "bfs":
                    next_id = select_next_target_bfs(graph, from_id, self._blocked_edges)
                else:
                    next_id = select_next_target_dfs(
                        graph,
                        from_id,
                        self._blocked_edges,
                        self._unexplored_bonus,
                        self._revisit_penalty,
                        self._cell_commit_confidence,
                        self._cell_dominance_margin,
                        self._cell_unvisited_bonus_gain if self._cell_memory_enabled else 0.0,
                        self._cell_revisit_penalty_gain if self._cell_memory_enabled else 0.0,
                        self._cell_opening_bonus_gain if self._cell_memory_enabled else 0.0,
                    )
                self._target_node_id = next_id

            if self._target_node_id is not None and current_id is not None:
                self._current_path = build_path_to_node(
                    graph, current_id, self._target_node_id, self._blocked_edges
                )
            if self._target_node_id is not None:
                target_x_m, target_y_m = self._regulated_target_position(
                    self._target_node_id,
                    current_id=current_id,
                    path_ids=self._current_path,
                )
                self._best_target_distance_m = distance_m(rx, ry, target_x_m, target_y_m)
                self._last_progress_time = time.time()
            else:
                self._best_target_distance_m = None
                self._last_progress_time = None

        # 4. Publish
        if self._target_node_id is not None:
            target_node = graph.nodes.get(self._target_node_id)
            if target_node is not None:
                target_x_m, target_y_m = self._regulated_target_position(
                    self._target_node_id,
                    current_id=current_id,
                    path_ids=self._current_path,
                )
                self._publish_target(target_x_m, target_y_m)

        if self._current_path:
            self._publish_path(self._current_path)

    # ------------------------------------------------------------------
    def _heading_for_path_index(self, node_ids: List[int], index: int) -> Optional[float]:
        if self._graph is None or index < 0 or index >= len(node_ids):
            return None

        current = self._graph.nodes.get(node_ids[index])
        if current is None:
            return None

        if 0 < index < len(node_ids) - 1:
            previous = self._graph.nodes.get(node_ids[index - 1])
            following = self._graph.nodes.get(node_ids[index + 1])
            if previous is not None and following is not None:
                return heading_between_points(
                    previous.x_m, previous.y_m, following.x_m, following.y_m
                )

        if index < len(node_ids) - 1:
            following = self._graph.nodes.get(node_ids[index + 1])
            if following is not None:
                return heading_between_points(
                    current.x_m, current.y_m, following.x_m, following.y_m
                )

        if index > 0:
            previous = self._graph.nodes.get(node_ids[index - 1])
            if previous is not None:
                return heading_between_points(
                    previous.x_m, previous.y_m, current.x_m, current.y_m
                )

        robot_map_position = self._robot_position_in_map()
        if robot_map_position is None:
            return None
        robot_x_m, robot_y_m = robot_map_position
        return heading_between_points(robot_x_m, robot_y_m, current.x_m, current.y_m)

    def _regulated_point(
        self,
        x_m: float,
        y_m: float,
        heading_rad: Optional[float],
    ) -> Tuple[float, float]:
        if (
            not self._wall_centering_enabled
            or heading_rad is None
            or self._safe_free_map is None
            or self._map_resolution <= 0.0
        ):
            return x_m, y_m

        regulated_x_m, regulated_y_m, _ = regulate_target_between_walls(
            self._safe_free_map,
            self._map_origin_x,
            self._map_origin_y,
            self._map_resolution,
            x_m,
            y_m,
            heading_rad,
            self._wall_centering_max_lateral_offset_m,
            self._wall_centering_step_m,
            self._wall_centering_probe_distance_m,
            self._wall_centering_min_improvement_m,
            self._wall_centering_longitudinal_window_m,
            self._wall_centering_longitudinal_step_m,
            self._wall_centering_required_bounded_fraction,
        )
        return regulated_x_m, regulated_y_m

    def _regulated_target_position(
        self,
        target_node_id: int,
        current_id: Optional[int],
        path_ids: List[int],
    ) -> Tuple[float, float]:
        if self._graph is None:
            return 0.0, 0.0

        target_node = self._graph.nodes.get(target_node_id)
        if target_node is None:
            return 0.0, 0.0

        heading_rad: Optional[float] = None
        if path_ids and path_ids[-1] == target_node_id:
            heading_rad = self._heading_for_path_index(path_ids, len(path_ids) - 1)
        elif current_id is not None and current_id != target_node_id:
            current_node = self._graph.nodes.get(current_id)
            if current_node is not None:
                heading_rad = heading_between_points(
                    current_node.x_m,
                    current_node.y_m,
                    target_node.x_m,
                    target_node.y_m,
                )
        else:
            robot_map_position = self._robot_position_in_map()
            if robot_map_position is not None:
                robot_x_m, robot_y_m = robot_map_position
                heading_rad = heading_between_points(
                    robot_x_m,
                    robot_y_m,
                    target_node.x_m,
                    target_node.y_m,
                )

        return self._regulated_point(target_node.x_m, target_node.y_m, heading_rad)

    def _stalled_on_target(self, distance_to_target_m: float) -> bool:
        if (
            self._progress_stall_timeout_sec <= 0.0
            or self._progress_stall_min_improvement_m <= 0.0
        ):
            return False

        now = time.time()
        if self._last_progress_time is None or self._best_target_distance_m is None:
            self._last_progress_time = now
            self._best_target_distance_m = distance_to_target_m
            return False

        if distance_to_target_m <= (
            self._best_target_distance_m - self._progress_stall_min_improvement_m
        ):
            self._best_target_distance_m = distance_to_target_m
            self._last_progress_time = now
            return False

        return (now - self._last_progress_time) >= self._progress_stall_timeout_sec

    def _mark_current_path_blocked(
        self,
        current_id: Optional[int],
        reason: str,
    ) -> None:
        if self._graph is None:
            return

        edge_id: Optional[int] = None
        if len(self._current_path) >= 2:
            edge_id = find_edge_between_nodes(
                self._graph,
                self._current_path[0],
                self._current_path[1],
            )
        elif (
            current_id is not None
            and self._target_node_id is not None
            and current_id != self._target_node_id
        ):
            edge_id = find_edge_between_nodes(
                self._graph,
                current_id,
                self._target_node_id,
            )

        if edge_id is not None:
            edge = self._graph.edges.get(edge_id)
            if edge is not None:
                edge.blocked_count += 1
                self._blocked_edges.add(edge_id)
                self._blocked_times[edge_id] = time.time()
                if edge.blocked_count >= self._retry_limit:
                    edge.edge_state = EdgeState.BLOCKED
                self.get_logger().warn(
                    f'Marking edge {edge_id} blocked while targeting node '
                    f'{self._target_node_id}: {reason}'
                )
        else:
            self.get_logger().warn(
                f'Unable to identify an edge to block while targeting node '
                f'{self._target_node_id}: {reason}'
            )

        self._target_node_id = None
        self._current_path = []
        self._last_progress_time = None
        self._best_target_distance_m = None

    def _publish_target(self, target_x_m: float, target_y_m: float):
        ps = PoseStamped()
        ps.header.frame_id = self._map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = target_x_m
        ps.pose.position.y = target_y_m
        ps.pose.position.z = 0.0
        robot_map_position = self._robot_position_in_map()
        robot_x_m, robot_y_m = robot_map_position if robot_map_position is not None else (
            target_x_m,
            target_y_m,
        )
        yaw = heading_between_points(robot_x_m, robot_y_m, target_x_m, target_y_m)
        ps.pose.orientation = _yaw_to_quat(yaw)
        self._pub_target.publish(ps)

    def _publish_path(self, node_ids: List[int]):
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for index, nid in enumerate(node_ids):
            node = self._graph.nodes.get(nid)
            if node is None:
                continue
            ps = PoseStamped()
            ps.header = path.header
            point_x_m = node.x_m
            point_y_m = node.y_m
            if index > 0:
                heading_rad = self._heading_for_path_index(node_ids, index)
                point_x_m, point_y_m = self._regulated_point(
                    point_x_m,
                    point_y_m,
                    heading_rad,
                )
            ps.pose.position.x = point_x_m
            ps.pose.position.y = point_y_m
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._pub_path.publish(path)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MazeGraphPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
