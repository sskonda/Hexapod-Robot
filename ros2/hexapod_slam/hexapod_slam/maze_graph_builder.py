#!/usr/bin/env python3
"""
maze_graph_builder.py

Builds a persistent topological maze graph from the SLAM occupancy map.
Publishes /maze_graph/graph_json (JSON) plus debug MarkerArrays.
"""

import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from .graph_io import graph_to_json, load_graph, save_graph
from .graph_types import (
    EdgeState, GraphState, MazeEdge, MazeNode, NodeType, VisitState,
)
from .map_frontier_utils import (
    detect_frontiers,
    distance_m,
    grid_to_world,
    heading_between_points,
    inflate_obstacles,
    line_of_sight_is_free,
    make_binary_free,
    occupancy_grid_to_array,
    ray_cast_distance,
    world_to_grid,
)

# ---------------------------------------------------------------------------
# Pure-function API (used by both the Node and the test suite)
# ---------------------------------------------------------------------------

NUM_RAYS = 16
RAY_ANGLES = [i * (2.0 * math.pi / NUM_RAYS) for i in range(NUM_RAYS)]


def extract_graph_candidates(
    binary_free: np.ndarray,
    safe_free: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    candidate_grid_spacing_m: float,
    probe_distance_m: float,
    corner_angle_threshold_deg: float,
    node_merge_distance_m: float,
) -> List[Tuple[float, float, NodeType]]:
    """
    Sample a grid of candidate points, classify them, and return
    a list of (x_m, y_m, NodeType) after intra-scan merging.

    This is a standalone function so it can be unit-tested without ROS 2.
    """
    h, w = safe_free.shape
    spacing_cells = max(1, int(round(candidate_grid_spacing_m / resolution)))
    probe_cells = probe_distance_m / resolution
    merge_cells = node_merge_distance_m / resolution
    corner_thresh_rad = math.radians(corner_angle_threshold_deg)

    candidates: List[Tuple[float, float, NodeType]] = []

    gx_vals = range(0, w, spacing_cells)
    gy_vals = range(0, h, spacing_cells)

    for gy in gy_vals:
        for gx in gx_vals:
            if not safe_free[gy, gx]:
                continue

            # Cast 16 rays
            open_rays: List[float] = []
            for angle in RAY_ANGLES:
                dist = ray_cast_distance(safe_free, gx, gy, angle, probe_cells)
                if dist >= probe_cells * 0.9:
                    open_rays.append(angle)

            n_open = len(open_rays)

            if n_open == 0:
                continue

            node_type: Optional[NodeType] = None

            if n_open == 1:
                node_type = NodeType.DEAD_END

            elif n_open == 2:
                a0, a1 = open_rays[0], open_rays[1]
                diff = abs(a1 - a0)
                diff = min(diff, 2.0 * math.pi - diff)
                if diff > corner_thresh_rad:
                    node_type = NodeType.CORNER

            else:  # 3+
                node_type = NodeType.JUNCTION

            if node_type is None:
                continue

            # World coords
            x_m, y_m = grid_to_world(gx, gy, origin_x, origin_y, resolution)

            # Merge with already-found candidates
            merged = False
            for i, (cx, cy, ctype) in enumerate(candidates):
                if distance_m(x_m, y_m, cx, cy) < merge_cells * resolution:
                    # Keep existing centre (prevents cascade drift), prefer higher-priority type
                    # Priority: JUNCTION > CORNER > DEAD_END > FRONTIER
                    priority = {
                        NodeType.JUNCTION: 3,
                        NodeType.CORNER: 2,
                        NodeType.DEAD_END: 1,
                        NodeType.FRONTIER: 0,
                        NodeType.GOAL_CANDIDATE: 0,
                        NodeType.START: 0,
                    }
                    best = ctype if priority[ctype] >= priority[node_type] else node_type
                    candidates[i] = (cx, cy, best)
                    merged = True
                    break

            if not merged:
                candidates.append((x_m, y_m, node_type))

    return candidates


def update_graph_nodes(
    graph: GraphState,
    candidates: List[Tuple[float, float, NodeType]],
    node_merge_distance_m: float,
    min_feature_observations: int,
    seen_node_ids: set,
) -> int:
    """
    Merge candidates into graph.nodes. Returns the next available node id
    (for external tracking, pass graph's current max id + 1 as the seed).
    """
    next_id = (max(graph.nodes.keys()) + 1) if graph.nodes else 0
    now = time.time()

    priority = {
        NodeType.JUNCTION: 3,
        NodeType.CORNER: 2,
        NodeType.DEAD_END: 1,
        NodeType.FRONTIER: 0,
        NodeType.GOAL_CANDIDATE: 0,
        NodeType.START: 0,
    }

    for x_m, y_m, node_type in candidates:
        # Find closest existing node
        best_id: Optional[int] = None
        best_dist = float("inf")
        for nid, node in graph.nodes.items():
            d = distance_m(x_m, y_m, node.x_m, node.y_m)
            if d < best_dist:
                best_dist = d
                best_id = nid

        if best_id is not None and best_dist <= node_merge_distance_m:
            node = graph.nodes[best_id]
            # Rolling average position
            n = node.observation_count
            node.x_m = (node.x_m * n + x_m) / (n + 1)
            node.y_m = (node.y_m * n + y_m) / (n + 1)
            node.observation_count += 1
            node.last_seen_time = now
            if priority.get(node_type, 0) > priority.get(node.node_type, 0):
                node.node_type = node_type
            if node.observation_count >= min_feature_observations:
                if node.visit_state == VisitState.UNSEEN:
                    node.visit_state = VisitState.SEEN
            node.confidence = min(1.0, node.observation_count / 5.0)
            seen_node_ids.add(best_id)
        else:
            new_node = MazeNode(
                id=next_id,
                x_m=x_m,
                y_m=y_m,
                node_type=node_type,
                visit_state=VisitState.UNSEEN,
                observation_count=1,
                last_seen_time=now,
                confidence=0.2,
            )
            graph.nodes[next_id] = new_node
            seen_node_ids.add(next_id)
            next_id += 1

    return next_id


def build_edges(
    graph: GraphState,
    binary_free: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    max_edge_length_m: float = 2.0,
) -> int:
    """
    For each pair of nodes within max_edge_length_m, check line-of-sight
    and create an edge if one doesn't already exist.
    Returns the next available edge id.
    """
    next_eid = (max(graph.edges.keys()) + 1) if graph.edges else 0
    node_list = list(graph.nodes.values())

    # Build a set of existing (start, end) pairs for quick lookup
    existing: set = set()
    for edge in graph.edges.values():
        existing.add((edge.start_node_id, edge.end_node_id))
        existing.add((edge.end_node_id, edge.start_node_id))

    for i, n1 in enumerate(node_list):
        for j in range(i + 1, len(node_list)):
            n2 = node_list[j]
            d = distance_m(n1.x_m, n1.y_m, n2.x_m, n2.y_m)
            if d > max_edge_length_m:
                continue
            if (n1.id, n2.id) in existing:
                continue

            # Grid coords
            gx1, gy1 = world_to_grid(n1.x_m, n1.y_m, origin_x, origin_y, resolution)
            gx2, gy2 = world_to_grid(n2.x_m, n2.y_m, origin_x, origin_y, resolution)

            if not line_of_sight_is_free(binary_free, gx1, gy1, gx2, gy2):
                continue

            heading = heading_between_points(n1.x_m, n1.y_m, n2.x_m, n2.y_m)
            edge = MazeEdge(
                id=next_eid,
                start_node_id=n1.id,
                end_node_id=n2.id,
                polyline_points=[[n1.x_m, n1.y_m], [n2.x_m, n2.y_m]],
                length_m=d,
                heading_rad=heading,
                edge_state=EdgeState.UNEXPLORED,
                confidence=min(n1.confidence, n2.confidence),
            )
            graph.edges[next_eid] = edge
            n1.connected_edge_ids.append(next_eid)
            n2.connected_edge_ids.append(next_eid)
            existing.add((n1.id, n2.id))
            existing.add((n2.id, n1.id))
            next_eid += 1

    return next_eid


def add_frontier_nodes(
    graph: GraphState,
    frontier_cells: List[Tuple[int, int]],
    origin_x: float,
    origin_y: float,
    resolution: float,
    merge_distance_m: float,
    next_node_id: int,
) -> int:
    """Convert detected frontier cells into FRONTIER nodes in the graph."""
    now = time.time()

    # Cluster frontier cells first by proximity
    clusters: List[List[Tuple[int, int]]] = []
    for gx, gy in frontier_cells:
        x_m, y_m = grid_to_world(gx, gy, origin_x, origin_y, resolution)
        placed = False
        for cluster in clusters:
            cx, cy = grid_to_world(cluster[0][0], cluster[0][1], origin_x, origin_y, resolution)
            if distance_m(x_m, y_m, cx, cy) < merge_distance_m * 2:
                cluster.append((gx, gy))
                placed = True
                break
        if not placed:
            clusters.append([(gx, gy)])

    for cluster in clusters:
        xs = [grid_to_world(gx, gy, origin_x, origin_y, resolution)[0] for gx, gy in cluster]
        ys = [grid_to_world(gx, gy, origin_x, origin_y, resolution)[1] for gx, gy in cluster]
        cx_m = float(np.mean(xs))
        cy_m = float(np.mean(ys))

        # Check if already represented by an existing node
        already_present = False
        for node in graph.nodes.values():
            if distance_m(cx_m, cy_m, node.x_m, node.y_m) < merge_distance_m:
                already_present = True
                break

        if already_present:
            continue

        graph.nodes[next_node_id] = MazeNode(
            id=next_node_id,
            x_m=cx_m,
            y_m=cy_m,
            node_type=NodeType.FRONTIER,
            visit_state=VisitState.UNSEEN,
            observation_count=1,
            last_seen_time=now,
            confidence=0.1,
        )
        next_node_id += 1

    return next_node_id


# ---------------------------------------------------------------------------
# Marker generation (standalone so tests can call it)
# ---------------------------------------------------------------------------

_NODE_COLORS: Dict[NodeType, Tuple[float, float, float]] = {
    NodeType.FRONTIER:       (1.0, 1.0, 0.0),
    NodeType.CORNER:         (0.0, 0.9, 0.9),
    NodeType.JUNCTION:       (0.0, 0.3, 1.0),
    NodeType.DEAD_END:       (0.9, 0.0, 0.0),
    NodeType.START:          (1.0, 1.0, 1.0),
    NodeType.GOAL_CANDIDATE: (1.0, 0.5, 0.0),
}


def generate_node_markers(
    graph: GraphState,
    frame_id: str,
    node_scale_m: float = 0.10,
) -> MarkerArray:
    """Generate SPHERE markers for all nodes, coloured by NodeType."""
    ma = MarkerArray()
    for i, node in enumerate(graph.nodes.values()):
        r, g, b = _NODE_COLORS.get(node.node_type, (0.8, 0.8, 0.8))
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = "maze_nodes"
        m.id = node.id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = node.x_m
        m.pose.position.y = node.y_m
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = node_scale_m
        m.scale.y = node_scale_m
        m.scale.z = node_scale_m
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.9
        ma.markers.append(m)
    return ma


def generate_edge_markers(
    graph: GraphState,
    frame_id: str,
    edge_width_m: float = 0.03,
) -> MarkerArray:
    """Generate LINE_STRIP markers for all edges, coloured by EdgeState."""
    _edge_colors = {
        EdgeState.UNEXPLORED: (0.5, 0.5, 0.5),
        EdgeState.OPEN:       (0.0, 0.8, 0.0),
        EdgeState.VISITED:    (0.0, 0.8, 0.0),
        EdgeState.BLOCKED:    (1.0, 0.5, 0.0),
    }
    ma = MarkerArray()
    for edge in graph.edges.values():
        r, g, b = _edge_colors.get(edge.edge_state, (0.5, 0.5, 0.5))
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = "maze_edges"
        m.id = edge.id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = edge_width_m
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.8
        for pt in edge.polyline_points:
            p = Point()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = 0.02
            m.points.append(p)
        ma.markers.append(m)
    return ma


def generate_frontier_markers(
    frontier_cells: List[Tuple[int, int]],
    origin_x: float,
    origin_y: float,
    resolution: float,
    frame_id: str,
    scale_m: float = 0.05,
) -> MarkerArray:
    """Generate a POINTS marker for raw frontier cells."""
    ma = MarkerArray()
    if not frontier_cells:
        return ma
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = "maze_frontiers"
    m.id = 0
    m.type = Marker.POINTS
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = scale_m
    m.scale.y = scale_m
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.color.a = 0.8
    for gx, gy in frontier_cells:
        x_m, y_m = grid_to_world(gx, gy, origin_x, origin_y, resolution)
        p = Point()
        p.x = x_m
        p.y = y_m
        p.z = 0.01
        m.points.append(p)
    ma.markers.append(m)
    return ma


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class MazeGraphBuilder(Node):
    def __init__(self):
        super().__init__("maze_graph_builder")

        # --- parameters ---
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("update_rate_hz", 2.0)
        self.declare_parameter("min_feature_observations", 2)
        self.declare_parameter("node_merge_distance_m", 0.25)
        self.declare_parameter("min_node_spacing_m", 0.30)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("free_threshold", 20)
        self.declare_parameter("inflation_radius_m", 0.12)
        self.declare_parameter("robot_clearance_radius_m", 0.15)
        self.declare_parameter("corner_angle_threshold_deg", 40.0)
        self.declare_parameter("probe_distance_m", 0.50)
        self.declare_parameter("candidate_grid_spacing_m", 0.20)
        self.declare_parameter("min_corridor_width_m", 0.20)
        self.declare_parameter("graph_save_path", "/tmp/maze_graph.json")
        self.declare_parameter("graph_autosave_period_sec", 30.0)

        self._map_frame = self.get_parameter("map_frame").value
        self._update_rate = self.get_parameter("update_rate_hz").value
        self._min_obs = self.get_parameter("min_feature_observations").value
        self._merge_dist = self.get_parameter("node_merge_distance_m").value
        self._free_thresh = self.get_parameter("free_threshold").value
        self._inflation_m = self.get_parameter("inflation_radius_m").value
        self._corner_thresh = self.get_parameter("corner_angle_threshold_deg").value
        self._probe_dist = self.get_parameter("probe_distance_m").value
        self._spacing_m = self.get_parameter("candidate_grid_spacing_m").value
        self._save_path = self.get_parameter("graph_save_path").value
        self._autosave_period = self.get_parameter("graph_autosave_period_sec").value

        # --- state ---
        self._graph = GraphState()
        self._last_map: Optional[OccupancyGrid] = None
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._last_save_time: float = time.time()
        self._last_frontier_cells: List[Tuple[int, int]] = []

        # --- subscriptions ---
        self.create_subscription(OccupancyGrid, "/map", self._map_callback, 1)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)

        # --- publishers ---
        self._pub_json = self.create_publisher(String, "/maze_graph/graph_json", 1)
        self._pub_nodes = self.create_publisher(MarkerArray, "/maze_graph/nodes", 1)
        self._pub_edges = self.create_publisher(MarkerArray, "/maze_graph/edges", 1)
        self._pub_frontiers = self.create_publisher(MarkerArray, "/maze_graph/frontiers", 1)

        # --- timer ---
        period = 1.0 / max(0.1, self._update_rate)
        self.create_timer(period, self._timer_callback)

        self.get_logger().info("MazeGraphBuilder started.")

    # ------------------------------------------------------------------
    def _map_callback(self, msg: OccupancyGrid):
        self._last_map = msg

    def _odom_callback(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    # ------------------------------------------------------------------
    def _timer_callback(self):
        if self._last_map is None:
            return

        msg = self._last_map
        resolution = msg.info.resolution
        if resolution <= 0.0:
            return

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        try:
            grid = occupancy_grid_to_array(msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse map: {exc}")
            return

        self._extract_graph_from_map(grid, origin_x, origin_y, resolution)

        # Publish
        json_str = graph_to_json(self._graph)
        msg_out = String()
        msg_out.data = json_str
        self._pub_json.publish(msg_out)

        node_markers = generate_node_markers(self._graph, self._map_frame)
        self._stamp_markers(node_markers)
        self._pub_nodes.publish(node_markers)

        edge_markers = generate_edge_markers(self._graph, self._map_frame)
        self._stamp_markers(edge_markers)
        self._pub_edges.publish(edge_markers)

        frontier_markers = generate_frontier_markers(
            self._last_frontier_cells,
            origin_x,
            origin_y,
            resolution,
            self._map_frame,
        )
        self._stamp_markers(frontier_markers)
        self._pub_frontiers.publish(frontier_markers)

        # Autosave
        now = time.time()
        if now - self._last_save_time >= self._autosave_period:
            try:
                save_graph(self._graph, self._save_path)
                self._last_save_time = now
            except Exception as exc:
                self.get_logger().warn(f"Autosave failed: {exc}")

    def _stamp_markers(self, ma: MarkerArray):
        stamp = self.get_clock().now().to_msg()
        for m in ma.markers:
            m.header.stamp = stamp

    # ------------------------------------------------------------------
    def _extract_graph_from_map(
        self,
        grid_array: np.ndarray,
        origin_x: float,
        origin_y: float,
        resolution: float,
    ):
        """
        Full graph extraction pipeline (see architecture doc).
        """
        # 1. Binary free map
        binary_free = make_binary_free(grid_array, self._free_thresh)

        # 2. Inflate obstacles
        radius_cells = max(1, int(math.ceil(self._inflation_m / resolution)))
        safe_free = inflate_obstacles(binary_free, radius_cells)

        # 3+4. Extract and classify candidates
        candidates = extract_graph_candidates(
            binary_free=binary_free,
            safe_free=safe_free,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            candidate_grid_spacing_m=self._spacing_m,
            probe_distance_m=self._probe_dist,
            corner_angle_threshold_deg=self._corner_thresh,
            node_merge_distance_m=self._merge_dist,
        )

        # 5+6. Update persistent nodes
        seen_ids: set = set()
        update_graph_nodes(
            self._graph,
            candidates,
            self._merge_dist,
            self._min_obs,
            seen_ids,
        )

        # Remove nodes unseen in this pass with very low confidence
        stale_ids = [
            nid for nid, node in self._graph.nodes.items()
            if nid not in seen_ids and node.observation_count < 2
            and node.node_type != NodeType.START
        ]
        for nid in stale_ids:
            # Remove edges first
            for eid in list(self._graph.nodes[nid].connected_edge_ids):
                edge = self._graph.edges.pop(eid, None)
                if edge:
                    other_id = edge.end_node_id if edge.start_node_id == nid else edge.start_node_id
                    other = self._graph.nodes.get(other_id)
                    if other and eid in other.connected_edge_ids:
                        other.connected_edge_ids.remove(eid)
                    if eid in self._graph.frontier_edges:
                        self._graph.frontier_edges.remove(eid)
            del self._graph.nodes[nid]

        # 7. Build / update edges
        build_edges(
            self._graph,
            binary_free,
            origin_x,
            origin_y,
            resolution,
        )

        # 8. Detect frontiers and add FRONTIER nodes
        frontier_cells = detect_frontiers(grid_array, self._free_thresh)
        # Subsample to avoid flooding with frontier nodes
        step = max(1, len(frontier_cells) // 50)
        sampled_frontiers = frontier_cells[::step]
        self._last_frontier_cells = sampled_frontiers

        next_id = (max(self._graph.nodes.keys()) + 1) if self._graph.nodes else 0
        add_frontier_nodes(
            self._graph,
            sampled_frontiers,
            origin_x,
            origin_y,
            resolution,
            self._merge_dist,
            next_id,
        )

        self._graph.version += 1
        self._graph.timestamp = time.time()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MazeGraphBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
