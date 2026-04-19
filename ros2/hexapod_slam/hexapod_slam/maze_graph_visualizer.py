#!/usr/bin/env python3
"""
maze_graph_visualizer.py

Subscribes to /maze_graph/graph_json and publishes rich MarkerArrays for RViz.
"""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .graph_io import json_to_graph
from .graph_types import CellSideState, EdgeState, GraphState, MazeCell, MazeNode, NodeType, VisitState
from .maze_cell_memory import SIDE_NAMES, cell_wall_segment, classify_cell_side


# ---------------------------------------------------------------------------
# Standalone marker-generation functions (testable without ROS2)
# ---------------------------------------------------------------------------

_NODE_COLORS: Dict[str, Tuple[float, float, float]] = {
    "FRONTIER":       (1.0, 1.0, 0.0),
    "CORNER":         (0.0, 0.9, 0.9),
    "JUNCTION":       (0.0, 0.3, 1.0),
    "DEAD_END":       (0.9, 0.0, 0.0),
    "START":          (1.0, 1.0, 1.0),
    "GOAL_CANDIDATE": (1.0, 0.5, 0.0),
    "VISITED":        (0.0, 0.8, 0.0),   # override for visit_state
}

_EDGE_COLORS: Dict[str, Tuple[float, float, float]] = {
    "UNEXPLORED": (0.5, 0.5, 0.5),
    "OPEN":       (0.0, 0.8, 0.0),
    "VISITED":    (0.0, 0.8, 0.0),
    "BLOCKED":    (1.0, 0.5, 0.0),
}


def _node_color(node: MazeNode) -> Tuple[float, float, float]:
    if node.visit_state == VisitState.VISITED:
        return _NODE_COLORS["VISITED"]
    return _NODE_COLORS.get(node.node_type.value, (0.8, 0.8, 0.8))


def generate_combined_markers(
    graph: GraphState,
    frame_id: str,
    node_scale_m: float = 0.08,
    edge_width_m: float = 0.03,
    active_target_id: Optional[int] = None,
) -> MarkerArray:
    """
    Returns a MarkerArray containing SPHERE markers for nodes and
    LINE_STRIP markers for edges, all in one array.
    """
    ma = MarkerArray()

    # Nodes
    for node in graph.nodes.values():
        r, g, b = _node_color(node)
        scale = node_scale_m
        if active_target_id is not None and node.id == active_target_id:
            r, g, b = 1.0, 0.0, 1.0  # magenta
            scale = node_scale_m * 1.6

        m = Marker()
        m.header.frame_id = frame_id
        m.ns = "viz_nodes"
        m.id = node.id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = node.x_m
        m.pose.position.y = node.y_m
        m.pose.position.z = 0.06
        m.pose.orientation.w = 1.0
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.9
        ma.markers.append(m)

    # Edges
    for edge in graph.edges.values():
        r, g, b = _EDGE_COLORS.get(edge.edge_state.value, (0.5, 0.5, 0.5))
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = "viz_edges"
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
            p.z = 0.03
            m.points.append(p)
        ma.markers.append(m)

    return ma


def generate_label_markers(
    graph: GraphState,
    frame_id: str,
    text_scale_m: float = 0.10,
) -> MarkerArray:
    """TEXT_VIEW_FACING markers showing node id and type."""
    ma = MarkerArray()
    for node in graph.nodes.values():
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = "viz_labels"
        m.id = node.id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = node.x_m
        m.pose.position.y = node.y_m
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.z = text_scale_m
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = f"{node.id}:{node.node_type.value[:3]}"
        ma.markers.append(m)
    return ma


def generate_frontier_markers_viz(
    graph: GraphState,
    frame_id: str,
    scale_m: float = 0.05,
) -> MarkerArray:
    """SPHERE markers for all FRONTIER-type nodes."""
    ma = MarkerArray()
    frontier_nodes = [n for n in graph.nodes.values() if n.node_type == NodeType.FRONTIER]
    for i, node in enumerate(frontier_nodes):
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = "viz_frontiers"
        m.id = i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = node.x_m
        m.pose.position.y = node.y_m
        m.pose.position.z = 0.04
        m.pose.orientation.w = 1.0
        m.scale.x = scale_m
        m.scale.y = scale_m
        m.scale.z = scale_m
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.85
        ma.markers.append(m)
    return ma


def generate_active_target_marker(
    graph: GraphState,
    frame_id: str,
    scale_m: float = 0.12,
) -> MarkerArray:
    """Magenta CYLINDER marker for the active target node."""
    ma = MarkerArray()
    tid = graph.active_target_node_id
    if tid is None or tid not in graph.nodes:
        return ma
    node = graph.nodes[tid]
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = "viz_active_target"
    m.id = 0
    m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.pose.position.x = node.x_m
    m.pose.position.y = node.y_m
    m.pose.position.z = 0.10
    m.pose.orientation.w = 1.0
    m.scale.x = scale_m
    m.scale.y = scale_m
    m.scale.z = 0.20
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 1.0
    m.color.a = 0.85
    ma.markers.append(m)
    return ma


def _cell_fill_color(cell: MazeCell) -> Tuple[float, float, float, float]:
    if cell.visit_count > 0:
        return 0.1, 0.8, 0.2, min(0.65, 0.20 + 0.10 * cell.visit_count)
    return 0.1, 0.6, 1.0, min(0.45, 0.10 + 0.35 * cell.confidence)


def generate_cell_markers(
    graph: GraphState,
    frame_id: str,
    cell_scale_ratio: float = 0.25,
    wall_width_m: float = 0.03,
    commit_confidence: float = 0.65,
    dominance_margin: float = 0.15,
    show_openings: bool = True,
) -> MarkerArray:
    ma = MarkerArray()
    cell_size_m = graph.cell_size_m
    if cell_size_m <= 0.0 or not graph.cells:
        return ma

    for marker_id, cell in enumerate(graph.cells.values()):
        fill_r, fill_g, fill_b, fill_a = _cell_fill_color(cell)
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "viz_cells"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = cell.center_x_m
        marker.pose.position.y = cell.center_y_m
        marker.pose.position.z = 0.015
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.04, cell_size_m * cell_scale_ratio)
        marker.scale.y = max(0.04, cell_size_m * cell_scale_ratio)
        marker.scale.z = 0.02
        marker.color.r = fill_r
        marker.color.g = fill_g
        marker.color.b = fill_b
        marker.color.a = fill_a
        ma.markers.append(marker)

    wall_marker = Marker()
    wall_marker.header.frame_id = frame_id
    wall_marker.ns = "viz_cell_walls"
    wall_marker.id = 0
    wall_marker.type = Marker.LINE_LIST
    wall_marker.action = Marker.ADD
    wall_marker.pose.orientation.w = 1.0
    wall_marker.scale.x = wall_width_m
    wall_marker.color.r = 1.0
    wall_marker.color.g = 0.4
    wall_marker.color.b = 0.1
    wall_marker.color.a = 0.9

    opening_marker = Marker()
    opening_marker.header.frame_id = frame_id
    opening_marker.ns = "viz_cell_openings"
    opening_marker.id = 1
    opening_marker.type = Marker.LINE_LIST
    opening_marker.action = Marker.ADD
    opening_marker.pose.orientation.w = 1.0
    opening_marker.scale.x = max(0.5 * wall_width_m, 0.015)
    opening_marker.color.r = 0.1
    opening_marker.color.g = 0.9
    opening_marker.color.b = 0.2
    opening_marker.color.a = 0.9

    for cell in graph.cells.values():
        for side_index, _ in enumerate(SIDE_NAMES):
            side_state = classify_cell_side(cell, side_index, commit_confidence, dominance_margin)
            if side_state == CellSideState.UNKNOWN:
                continue
            if side_state == CellSideState.OPEN and not show_openings:
                continue
            (start_x_m, start_y_m), (end_x_m, end_y_m) = cell_wall_segment(
                cell,
                side_index,
                cell_size_m,
            )
            start_point = Point()
            start_point.x = start_x_m
            start_point.y = start_y_m
            start_point.z = 0.04 if side_state == CellSideState.WALL else 0.03

            end_point = Point()
            end_point.x = end_x_m
            end_point.y = end_y_m
            end_point.z = start_point.z

            if side_state == CellSideState.WALL:
                wall_marker.points.extend([start_point, end_point])
            else:
                opening_marker.points.extend([start_point, end_point])

    if wall_marker.points:
        ma.markers.append(wall_marker)
    if opening_marker.points:
        ma.markers.append(opening_marker)
    return ma


def generate_cell_label_markers(
    graph: GraphState,
    frame_id: str,
    text_scale_m: float = 0.08,
    min_confidence: float = 0.35,
) -> MarkerArray:
    ma = MarkerArray()
    for marker_id, cell in enumerate(graph.cells.values()):
        if cell.confidence < min_confidence and cell.visit_count <= 0:
            continue
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "viz_cell_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = cell.center_x_m
        marker.pose.position.y = cell.center_y_m
        marker.pose.position.z = 0.10
        marker.pose.orientation.w = 1.0
        marker.scale.z = text_scale_m
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.text = f"{cell.grid_x},{cell.grid_y} v{cell.visit_count}"
        ma.markers.append(marker)
    return ma


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class MazeGraphVisualizer(Node):
    def __init__(self):
        super().__init__("maze_graph_visualizer")

        self.declare_parameter("marker_frame", "map")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("node_scale_m", 0.08)
        self.declare_parameter("edge_width_m", 0.03)
        self.declare_parameter("frontier_scale_m", 0.05)
        self.declare_parameter("label_text_scale_m", 0.10)
        self.declare_parameter("cell_label_text_scale_m", 0.08)
        self.declare_parameter("show_node_labels", True)
        self.declare_parameter("show_edge_labels", False)
        self.declare_parameter("show_frontiers", True)
        self.declare_parameter("show_dead_ends", True)
        self.declare_parameter("show_active_target", True)
        self.declare_parameter("show_cells", True)
        self.declare_parameter("show_cell_labels", True)
        self.declare_parameter("show_cell_openings", True)
        self.declare_parameter("cell_scale_ratio", 0.25)
        self.declare_parameter("cell_wall_width_m", 0.03)
        self.declare_parameter("cell_commit_confidence", 0.65)
        self.declare_parameter("cell_dominance_margin", 0.15)
        self.declare_parameter("cell_label_min_confidence", 0.35)
        self.declare_parameter("show_debug_candidates", False)

        self._frame = self.get_parameter("marker_frame").value
        self._rate = self.get_parameter("publish_rate_hz").value
        self._node_scale = self.get_parameter("node_scale_m").value
        self._edge_width = self.get_parameter("edge_width_m").value
        self._frontier_scale = self.get_parameter("frontier_scale_m").value
        self._label_scale = self.get_parameter("label_text_scale_m").value
        self._cell_label_scale = self.get_parameter("cell_label_text_scale_m").value
        self._show_labels = self.get_parameter("show_node_labels").value
        self._show_frontiers = self.get_parameter("show_frontiers").value
        self._show_target = self.get_parameter("show_active_target").value
        self._show_cells = self.get_parameter("show_cells").value
        self._show_cell_labels = self.get_parameter("show_cell_labels").value
        self._show_cell_openings = self.get_parameter("show_cell_openings").value
        self._cell_scale_ratio = self.get_parameter("cell_scale_ratio").value
        self._cell_wall_width = self.get_parameter("cell_wall_width_m").value
        self._cell_commit_confidence = self.get_parameter("cell_commit_confidence").value
        self._cell_dominance_margin = self.get_parameter("cell_dominance_margin").value
        self._cell_label_min_confidence = self.get_parameter("cell_label_min_confidence").value

        self._last_graph: Optional[GraphState] = None
        self._last_markers: Optional[MarkerArray] = None
        self._last_labels: Optional[MarkerArray] = None
        self._last_frontiers: Optional[MarkerArray] = None
        self._last_target: Optional[MarkerArray] = None
        self._last_cells: Optional[MarkerArray] = None
        self._last_cell_labels: Optional[MarkerArray] = None

        # Subscription
        self.create_subscription(String, "/maze_graph/graph_json", self._graph_callback, 1)

        # Publishers
        self._pub_markers = self.create_publisher(MarkerArray, "/maze_graph/markers", 1)
        self._pub_labels = self.create_publisher(MarkerArray, "/maze_graph/labels", 1)
        self._pub_frontiers = self.create_publisher(MarkerArray, "/maze_graph/frontier_markers", 1)
        self._pub_target = self.create_publisher(MarkerArray, "/maze_graph/active_target_marker", 1)
        self._pub_cells = self.create_publisher(MarkerArray, "/maze_graph/cell_markers", 1)
        self._pub_cell_labels = self.create_publisher(MarkerArray, "/maze_graph/cell_labels", 1)

        period = 1.0 / max(0.1, self._rate)
        self.create_timer(period, self._publish_timer)

        self.get_logger().info("MazeGraphVisualizer started.")

    def _graph_callback(self, msg: String):
        try:
            graph = json_to_graph(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse graph JSON: {exc}")
            return

        self._last_graph = graph
        stamp = self.get_clock().now().to_msg()

        self._last_markers = generate_combined_markers(
            graph, self._frame, self._node_scale, self._edge_width,
            graph.active_target_node_id,
        )
        self._stamp_ma(self._last_markers, stamp)

        if self._show_labels:
            self._last_labels = generate_label_markers(graph, self._frame, self._label_scale)
            self._stamp_ma(self._last_labels, stamp)

        if self._show_frontiers:
            self._last_frontiers = generate_frontier_markers_viz(
                graph, self._frame, self._frontier_scale
            )
            self._stamp_ma(self._last_frontiers, stamp)

        if self._show_target:
            self._last_target = generate_active_target_marker(graph, self._frame)
            self._stamp_ma(self._last_target, stamp)

        if self._show_cells:
            self._last_cells = generate_cell_markers(
                graph,
                self._frame,
                self._cell_scale_ratio,
                self._cell_wall_width,
                self._cell_commit_confidence,
                self._cell_dominance_margin,
                self._show_cell_openings,
            )
            self._stamp_ma(self._last_cells, stamp)

        if self._show_cell_labels:
            self._last_cell_labels = generate_cell_label_markers(
                graph,
                self._frame,
                self._cell_label_scale,
                self._cell_label_min_confidence,
            )
            self._stamp_ma(self._last_cell_labels, stamp)

        self._publish_all()

    def _publish_timer(self):
        self._publish_all()

    def _publish_all(self):
        if self._last_markers:
            self._pub_markers.publish(self._last_markers)
        if self._last_labels:
            self._pub_labels.publish(self._last_labels)
        if self._last_frontiers:
            self._pub_frontiers.publish(self._last_frontiers)
        if self._last_target:
            self._pub_target.publish(self._last_target)
        if self._last_cells:
            self._pub_cells.publish(self._last_cells)
        if self._last_cell_labels:
            self._pub_cell_labels.publish(self._last_cell_labels)

    def _stamp_ma(self, ma: MarkerArray, stamp):
        for m in ma.markers:
            m.header.stamp = stamp


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MazeGraphVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
