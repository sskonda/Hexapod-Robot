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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, String

from .graph_io import json_to_graph
from .graph_types import EdgeState, GraphState, MazeNode, NodeType, VisitState
from .map_frontier_utils import distance_m, heading_between_points


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
            if edge.id in blocked_edges:
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


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class MazeGraphPlanner(Node):
    def __init__(self):
        super().__init__("maze_graph_planner")

        self.declare_parameter("strategy", "dfs")
        self.declare_parameter("planner_rate_hz", 2.0)
        self.declare_parameter("target_reached_distance_m", 0.20)
        self.declare_parameter("current_node_snap_distance_m", 0.25)
        self.declare_parameter("blocked_edge_retry_limit", 2)
        self.declare_parameter("blocked_edge_cooldown_sec", 10.0)
        self.declare_parameter("unexplored_bonus_gain", 5.0)
        self.declare_parameter("revisit_penalty_gain", 1.0)
        self.declare_parameter("heading_alignment_gain", 0.5)
        self.declare_parameter("backtrack_penalty_gain", 0.5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self._strategy = self.get_parameter("strategy").value
        self._rate = self.get_parameter("planner_rate_hz").value
        self._target_dist = self.get_parameter("target_reached_distance_m").value
        self._snap_dist = self.get_parameter("current_node_snap_distance_m").value
        self._retry_limit = self.get_parameter("blocked_edge_retry_limit").value
        self._cooldown = self.get_parameter("blocked_edge_cooldown_sec").value
        self._unexplored_bonus = self.get_parameter("unexplored_bonus_gain").value
        self._revisit_penalty = self.get_parameter("revisit_penalty_gain").value
        self._map_frame = self.get_parameter("map_frame").value

        # State
        self._graph: Optional[GraphState] = None
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._target_node_id: Optional[int] = None
        self._current_path: List[int] = []
        self._blocked_edges: Set[int] = set()
        self._blocked_times: Dict[int, float] = {}

        # Subscriptions
        self.create_subscription(String, "/maze_graph/graph_json", self._graph_callback, 1)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)

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

    def _odom_callback(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

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

    def _plan(self):
        graph = self._graph
        rx, ry = self._robot_x, self._robot_y

        # 1. Find current node
        current_id = find_closest_node(graph, rx, ry, self._snap_dist)

        # 2. Check if target reached
        if self._target_node_id is not None:
            target_node = graph.nodes.get(self._target_node_id)
            if target_node is not None:
                dist = distance_m(rx, ry, target_node.x_m, target_node.y_m)
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
                        graph, from_id, self._blocked_edges,
                        self._unexplored_bonus, self._revisit_penalty,
                    )
                self._target_node_id = next_id

            if self._target_node_id is not None and current_id is not None:
                self._current_path = build_path_to_node(
                    graph, current_id, self._target_node_id, self._blocked_edges
                )

        # 4. Publish
        if self._target_node_id is not None:
            target_node = graph.nodes.get(self._target_node_id)
            if target_node is not None:
                self._publish_target(target_node)

        if self._current_path:
            self._publish_path(self._current_path)

    # ------------------------------------------------------------------
    def _publish_target(self, node: MazeNode):
        ps = PoseStamped()
        ps.header.frame_id = self._map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = node.x_m
        ps.pose.position.y = node.y_m
        ps.pose.position.z = 0.0
        yaw = heading_between_points(self._robot_x, self._robot_y, node.x_m, node.y_m)
        ps.pose.orientation = _yaw_to_quat(yaw)
        self._pub_target.publish(ps)

    def _publish_path(self, node_ids: List[int]):
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for nid in node_ids:
            node = self._graph.nodes.get(nid)
            if node is None:
                continue
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = node.x_m
            ps.pose.position.y = node.y_m
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
