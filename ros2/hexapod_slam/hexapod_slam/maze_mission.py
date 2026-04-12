#!/usr/bin/env python3
"""
maze_mission.py

Hexapod Mission 1: Maze Traversal

Traverses a physical maze built from 2-ft × 2-ft tiles by maintaining a
grid-based topological graph, classifying walls at each node with temporal
filtering, and executing DFS/BFS traversal with left-first branch ordering.

State machine
─────────────
  WAIT_FOR_DATA   → Sensors not ready yet; robot stands still.
  INIT_AXIS       → Waiting for first movement to fix "North" heading.
  AT_NODE         → Robot has arrived at a grid node; scan walls.
  CLASSIFYING     → Collecting N scans with temporal filter.
  PLANNING        → Choose next direction via DFS.
  TRAVERSING      → Following path to next grid node.
  BACKTRACKING    → Following BFS-computed path back toward an unvisited node.
  EXIT            → Open area detected; save graph and stop.
  DONE            → Mission finished.

Topics published
────────────────
  maze_mission/path          nav_msgs/Path           — consumed by crab_path_follower
  maze_mission/markers       visualization_msgs/MarkerArray — RViz graph

Topics subscribed
─────────────────
  scan_topic    (default /scan)   sensor_msgs/LaserScan
  odom_topic    (default odom)    nav_msgs/Odometry

Parameters
──────────
  scan_topic, odom_topic
  wall_threshold_m          0.38   — clearance < this → wall present
  open_threshold_m          0.55   — clearance > this → open passage
  exit_threshold_m          1.2192 — clearance > this (2 × tile) in ≥2 dirs → exit
  centering_tolerance_m     0.05   — must be within this of node centre before classifying
  classification_scans      5      — N scans that must agree for wall classification
  classification_ratio      0.8    — fraction of scans that must agree
  traversal_speed_mps       0.04   — used to estimate traversal timeout
  traversal_timeout_factor  3.0    — timeout = factor × (tile / speed)
  path_publish_rate_hz      10.0   — rate to republish path to keep follower alive
  scan_window_deg           15.0   — half-window for clearance measurement
  graph_save_path           /tmp/maze_graph.json
  use_bfs                   false  — true = BFS breadth-first, false = DFS depth-first
"""

from __future__ import annotations

import math
import os
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from hexapod_slam.grid_graph import (
    DIRECTIONS,
    DIRECTION_DELTA,
    DIRECTION_YAW,
    LEFT_FIRST_ORDER,
    OPPOSITE,
    TILE_SIZE_M,
    UNVISITED,
    VISITED,
    NODE_EXIT,
    GridGraph,
    save_graph_json,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) quaternion for a pure-yaw rotation."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def closest_cardinal(yaw_rad: float) -> str:
    """Return the cardinal direction closest to yaw_rad."""
    best = 'N'
    best_diff = float('inf')
    for d, target_yaw in DIRECTION_YAW.items():
        diff = abs(normalize_angle(yaw_rad - target_yaw))
        if diff < best_diff:
            best_diff = diff
            best = d
    return best


# ---------------------------------------------------------------------------
# State constants
# ---------------------------------------------------------------------------

S_WAIT_FOR_DATA = 'WAIT_FOR_DATA'
S_INIT_AXIS     = 'INIT_AXIS'
S_AT_NODE       = 'AT_NODE'
S_CLASSIFYING   = 'CLASSIFYING'
S_PLANNING      = 'PLANNING'
S_TRAVERSING    = 'TRAVERSING'
S_BACKTRACKING  = 'BACKTRACKING'
S_EXIT          = 'EXIT'
S_DONE          = 'DONE'

# Marker namespaces
NS_NODE   = 'maze_nodes'
NS_EDGE   = 'maze_edges'
NS_TARGET = 'maze_target'

# Node type → RGBA colour
NODE_COLORS: Dict[str, Tuple[float, float, float, float]] = {
    'unknown':   (0.5, 0.5, 0.5, 0.8),
    'corridor':  (0.2, 0.8, 0.2, 0.9),
    'junction':  (0.2, 0.4, 1.0, 0.9),
    'dead_end':  (1.0, 0.4, 0.2, 0.9),
    'exit':      (1.0, 1.0, 0.0, 1.0),
}


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class MazeMissionNode(Node):

    def __init__(self):
        super().__init__('maze_mission')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('wall_threshold_m', 0.38)
        self.declare_parameter('open_threshold_m', 0.55)
        self.declare_parameter('exit_threshold_m', TILE_SIZE_M * 2.0)
        self.declare_parameter('centering_tolerance_m', 0.05)
        self.declare_parameter('classification_scans', 5)
        self.declare_parameter('classification_ratio', 0.8)
        self.declare_parameter('traversal_speed_mps', 0.04)
        self.declare_parameter('traversal_timeout_factor', 3.0)
        self.declare_parameter('path_publish_rate_hz', 10.0)
        self.declare_parameter('scan_window_deg', 15.0)
        self.declare_parameter('graph_save_path', '/tmp/maze_graph.json')
        self.declare_parameter('use_bfs', False)
        self.declare_parameter('recovery_backup_m', 0.20)

        self._scan_topic      = str(self.get_parameter('scan_topic').value)
        self._odom_topic      = str(self.get_parameter('odom_topic').value)
        self._wall_thresh     = float(self.get_parameter('wall_threshold_m').value)
        self._open_thresh     = float(self.get_parameter('open_threshold_m').value)
        self._exit_thresh     = float(self.get_parameter('exit_threshold_m').value)
        self._center_tol      = float(self.get_parameter('centering_tolerance_m').value)
        self._cls_scans       = max(1, int(self.get_parameter('classification_scans').value))
        self._cls_ratio       = float(self.get_parameter('classification_ratio').value)
        self._speed           = max(0.01, float(self.get_parameter('traversal_speed_mps').value))
        self._timeout_factor  = float(self.get_parameter('traversal_timeout_factor').value)
        self._path_rate_hz    = float(self.get_parameter('path_publish_rate_hz').value)
        self._scan_window_rad = math.radians(max(1.0, float(self.get_parameter('scan_window_deg').value)))
        self._graph_save_path = str(self.get_parameter('graph_save_path').value)
        self._use_bfs         = bool(self.get_parameter('use_bfs').value)
        self._recovery_m      = float(self.get_parameter('recovery_backup_m').value)

        self._traversal_timeout_sec = self._timeout_factor * (TILE_SIZE_M / self._speed)
        self._path_period_sec = 1.0 / max(1.0, self._path_rate_hz)

        # ------------------------------------------------------------------
        # Publishers / Subscribers
        # ------------------------------------------------------------------
        self._path_pub = self.create_publisher(Path, 'maze_mission/path', 10)
        self._marker_pub = self.create_publisher(MarkerArray, 'maze_mission/markers', 10)

        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._scan_cb, 10)
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_cb, 10)

        self._timer = self.create_timer(1.0 / 20.0, self._tick)  # 20 Hz control

        # ------------------------------------------------------------------
        # Sensor state
        # ------------------------------------------------------------------
        self._scan: Optional[LaserScan] = None
        self._prev_scan: Optional[LaserScan] = None
        self._scan_time: Optional[float] = None

        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0
        self._odom_frame = 'odom'
        self._odom_time: Optional[float] = None

        # ------------------------------------------------------------------
        # Graph state
        # ------------------------------------------------------------------
        self._graph: Optional[GridGraph] = None
        self._current_ij: Optional[Tuple[int, int]] = None
        self._travel_dir: Optional[str] = None   # direction we last moved
        self._north_yaw: Optional[float] = None  # odom yaw that corresponds to 'N'

        # ------------------------------------------------------------------
        # Classification buffer
        # ------------------------------------------------------------------
        # Each entry is a dict {direction: bool} (True=wall, False=open)
        self._cls_buffer: deque = deque(maxlen=self._cls_scans)
        self._cls_scans_collected = 0

        # ------------------------------------------------------------------
        # Traversal state
        # ------------------------------------------------------------------
        self._target_ij: Optional[Tuple[int, int]] = None
        self._target_dir: Optional[str] = None
        self._traversal_start_time: Optional[float] = None
        self._last_path_time: Optional[float] = None

        # Backtrack path: list of (i, j) waypoints remaining
        self._backtrack_waypoints: List[Tuple[int, int]] = []
        self._backtrack_wp_idx = 0

        # ------------------------------------------------------------------
        # State machine
        # ------------------------------------------------------------------
        self._state = S_WAIT_FOR_DATA
        self._state_entry_time: Optional[float] = None

        # Track consecutive traversal failures to avoid infinite stuck loops
        self._stuck_count = 0
        self._max_stuck = 5

        self.get_logger().info(
            f'MazeMission started ({"BFS" if self._use_bfs else "DFS left-first"}), '
            f'tile={TILE_SIZE_M:.4f} m, '
            f'wall<{self._wall_thresh:.2f} m, open>{self._open_thresh:.2f} m, '
            f'exit>{self._exit_thresh:.2f} m'
        )

    # ======================================================================
    # Callbacks
    # ======================================================================

    def _scan_cb(self, msg: LaserScan):
        self._prev_scan = self._scan
        self._scan = msg
        self._scan_time = self.get_clock().now().nanoseconds / 1e9

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        self._odom_yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
        self._odom_frame = msg.header.frame_id or 'odom'
        self._odom_time = self.get_clock().now().nanoseconds / 1e9

    # ======================================================================
    # Main tick
    # ======================================================================

    def _tick(self):
        dispatch = {
            S_WAIT_FOR_DATA: self._state_wait_for_data,
            S_INIT_AXIS:     self._state_init_axis,
            S_AT_NODE:       self._state_at_node,
            S_CLASSIFYING:   self._state_classifying,
            S_PLANNING:      self._state_planning,
            S_TRAVERSING:    self._state_traversing,
            S_BACKTRACKING:  self._state_backtracking,
            S_EXIT:          self._state_exit,
            S_DONE:          self._state_done,
        }
        handler = dispatch.get(self._state)
        if handler is not None:
            handler()

    def _set_state(self, new_state: str):
        self.get_logger().info(f'State: {self._state} → {new_state}')
        self._state = new_state
        self._state_entry_time = self.get_clock().now().nanoseconds / 1e9

    # ======================================================================
    # Utility: sensor data freshness
    # ======================================================================

    def _data_ready(self) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        scan_ok = self._scan is not None and self._scan_time is not None and (now - self._scan_time) < 2.0
        odom_ok = self._odom_time is not None and (now - self._odom_time) < 2.0
        return scan_ok and odom_ok

    # ======================================================================
    # Utility: clearance measurement
    # ======================================================================

    def _cardinal_clearance(self, cardinal: str) -> float:
        """
        Minimum clearance in the given cardinal direction using the current
        (and previous) scan to filter leg interference.
        """
        if self._scan is None:
            return 0.0
        # Convert cardinal direction to base-frame angle
        map_yaw = DIRECTION_YAW[cardinal]
        base_angle = normalize_angle(map_yaw - self._odom_yaw)
        return self._clearance_at_base_angle(base_angle)

    def _clearance_at_base_angle(self, base_angle_rad: float) -> float:
        c_cur = self._window_clearance_scan(self._scan, base_angle_rad)
        if self._prev_scan is None:
            return c_cur if c_cur is not None else 0.0
        c_prev = self._window_clearance_scan(self._prev_scan, base_angle_rad)
        vals = [v for v in [c_cur, c_prev] if v is not None]
        return min(vals) if vals else 0.0

    def _window_clearance_scan(self, scan: LaserScan, base_angle_rad: float) -> Optional[float]:
        if scan is None or len(scan.ranges) == 0:
            return None
        angle_inc = abs(scan.angle_increment)
        if angle_inc < 1e-9:
            return None
        window_beams = max(1, int(self._scan_window_rad / angle_inc))
        raw_idx = (base_angle_rad - scan.angle_min) / scan.angle_increment
        center = int(round(raw_idx))
        center = max(window_beams, min(len(scan.ranges) - 1 - window_beams, center))
        min_c: Optional[float] = None
        for idx in range(center - window_beams, center + window_beams + 1):
            if idx < 0 or idx >= len(scan.ranges):
                continue
            r = scan.ranges[idx]
            if math.isnan(r):
                continue
            if math.isinf(r):
                r = scan.range_max
            elif r < scan.range_min:
                r = scan.range_min
            elif r > scan.range_max:
                r = scan.range_max
            if min_c is None or r < min_c:
                min_c = r
        return min_c

    def _all_cardinal_clearances(self) -> Dict[str, float]:
        return {d: self._cardinal_clearance(d) for d in DIRECTIONS}

    # ======================================================================
    # Utility: map frame ↔ robot heading
    # ======================================================================

    def _map_yaw_for_direction(self, cardinal: str) -> float:
        """Map-frame yaw corresponding to a cardinal direction given north_yaw."""
        if self._north_yaw is None:
            return DIRECTION_YAW[cardinal]
        # 'N' corresponds to north_yaw; rotate accordingly
        base = DIRECTION_YAW[cardinal]
        north_base = DIRECTION_YAW['N']
        return normalize_angle(self._north_yaw + (base - north_base))

    def _odom_direction(self, cardinal: str) -> str:
        """
        Returns the cardinal direction (in map frame) corresponding to the
        given mission cardinal direction after adjusting for north_yaw offset.
        Actually the grid is fixed to the odom frame, so this is an identity
        unless north_yaw differs from +Y.  We keep the grid aligned to the
        robot's initial heading for simplicity.
        """
        return cardinal

    # ======================================================================
    # Utility: robot ↔ grid position
    # ======================================================================

    def _snap_to_grid(self) -> Tuple[int, int]:
        """Return grid (i, j) nearest to current odom position."""
        assert self._graph is not None
        return self._graph.xy_to_ij(self._odom_x, self._odom_y)

    def _node_center_xy(self, ij: Tuple[int, int]) -> Tuple[float, float]:
        assert self._graph is not None
        return self._graph.ij_to_xy(*ij)

    def _dist_to_node_center(self, ij: Tuple[int, int]) -> float:
        cx, cy = self._node_center_xy(ij)
        return math.hypot(self._odom_x - cx, self._odom_y - cy)

    def _at_node_center(self, ij: Tuple[int, int]) -> bool:
        return self._dist_to_node_center(ij) <= self._center_tol

    # ======================================================================
    # Path publishing helpers
    # ======================================================================

    def _publish_path_to(self, target_x: float, target_y: float, heading_yaw: float):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._odom_frame
        path.poses.append(self._make_pose(self._odom_x, self._odom_y, heading_yaw))
        path.poses.append(self._make_pose(target_x, target_y, heading_yaw))
        self._path_pub.publish(path)
        self._last_path_time = self.get_clock().now().nanoseconds / 1e9

    def _publish_stop_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._odom_frame
        path.poses.append(self._make_pose(self._odom_x, self._odom_y, self._odom_yaw))
        self._path_pub.publish(path)
        self._last_path_time = self.get_clock().now().nanoseconds / 1e9

    def _should_refresh_path(self) -> bool:
        if self._last_path_time is None:
            return True
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self._last_path_time) >= self._path_period_sec

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self._odom_frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.0
        _, _, qz, qw = yaw_to_quat(yaw)
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    # ======================================================================
    # State: WAIT_FOR_DATA
    # ======================================================================

    def _state_wait_for_data(self):
        if self._data_ready():
            self.get_logger().info('Sensor data ready — initialising graph.')
            # Create graph with current position as origin (grid 0,0)
            self._graph = GridGraph(origin_x_m=self._odom_x, origin_y_m=self._odom_y)
            self._current_ij = (0, 0)
            self._graph.mark_visited(0, 0)
            self._set_state(S_INIT_AXIS)

    # ======================================================================
    # State: INIT_AXIS
    # ======================================================================

    def _state_init_axis(self):
        """
        Wait for the operator (or gap_following_explorer) to start moving the
        robot.  The direction of first significant movement defines 'North'.
        Meanwhile publish a stop path.
        """
        if not self._data_ready():
            return

        # Check if robot has moved enough to establish an axis
        origin_x, origin_y = self._node_center_xy((0, 0))
        dist = math.hypot(self._odom_x - origin_x, self._odom_y - origin_y)
        if dist > self._center_tol * 2.0:
            # The direction the robot is currently facing defines North
            self._north_yaw = self._odom_yaw
            self.get_logger().info(
                f'North axis fixed at yaw={math.degrees(self._north_yaw):.1f} deg'
            )
            self._set_state(S_AT_NODE)
        else:
            self._publish_stop_path()

    # ======================================================================
    # State: AT_NODE
    # ======================================================================

    def _state_at_node(self):
        if not self._data_ready():
            self._publish_stop_path()
            return

        if self._current_ij is None:
            self._current_ij = self._snap_to_grid()
            self._graph.mark_visited(*self._current_ij)

        # Check for exit before classifying
        clears = self._all_cardinal_clearances()
        if self._is_exit(clears):
            self._graph.set_exit(*self._current_ij)
            self.get_logger().info(
                f'EXIT detected at node {self._current_ij}! '
                f'Clearances: {", ".join(f"{d}={v:.2f}m" for d, v in clears.items())}'
            )
            self._set_state(S_EXIT)
            return

        # Start classifying walls
        self._cls_buffer.clear()
        self._cls_scans_collected = 0
        self._publish_stop_path()
        self._set_state(S_CLASSIFYING)

    # ======================================================================
    # State: CLASSIFYING
    # ======================================================================

    def _state_classifying(self):
        if not self._data_ready():
            self._publish_stop_path()
            return

        # Collect one scan's worth of readings
        clears = self._all_cardinal_clearances()
        reading: Dict[str, bool] = {}
        for d, c in clears.items():
            if c < self._wall_thresh:
                reading[d] = True   # wall
            elif c > self._open_thresh:
                reading[d] = False  # open
            # else ambiguous — don't add to reading; won't count toward either

        self._cls_buffer.append(reading)
        self._cls_scans_collected += 1

        if self._cls_scans_collected < self._cls_scans:
            # Keep robot stopped while collecting
            self._publish_stop_path()
            return

        # Enough scans — aggregate with ratio filter
        wall_readings: Dict[str, bool] = {}
        for d in DIRECTIONS:
            wall_votes = sum(1 for r in self._cls_buffer if r.get(d) is True)
            open_votes = sum(1 for r in self._cls_buffer if r.get(d) is False)
            n = wall_votes + open_votes
            if n == 0:
                continue
            if wall_votes / n >= self._cls_ratio:
                wall_readings[d] = True
            elif open_votes / n >= self._cls_ratio:
                wall_readings[d] = False

        if wall_readings:
            self._graph.record_walls(*self._current_ij, wall_readings)
            open_dirs = [d for d, v in wall_readings.items() if not v]
            wall_dirs = [d for d, v in wall_readings.items() if v]
            self.get_logger().info(
                f'Node {self._current_ij}: '
                f'open={open_dirs}, wall={wall_dirs}, '
                f'type={self._graph.get_node(*self._current_ij).node_type}'
            )
        else:
            self.get_logger().warn(
                f'Node {self._current_ij}: no confident wall readings after '
                f'{self._cls_scans} scans — proceeding to planning.'
            )

        # Add edges for newly discovered open directions
        if self._current_ij is not None:
            for d in self._graph.open_directions(*self._current_ij):
                ni, nj = self._graph.neighbor_ij(*self._current_ij, d)
                self._graph.add_edge(self._current_ij, (ni, nj), d)
                # Reverse edge (the neighbour can come back)
                self._graph.add_edge((ni, nj), self._current_ij, OPPOSITE[d])

        self._publish_markers()
        self._set_state(S_PLANNING)

    # ======================================================================
    # State: PLANNING
    # ======================================================================

    def _state_planning(self):
        if self._current_ij is None:
            self._set_state(S_AT_NODE)
            return

        if self._use_bfs:
            next_dir = self._bfs_next_direction()
        else:
            next_dir = self._graph.dfs_next_direction(
                self._current_ij, self._travel_dir, allow_revisit=False
            )

        if next_dir is not None:
            ni, nj = self._graph.neighbor_ij(*self._current_ij, next_dir)
            self._graph.get_or_create_node(ni, nj)  # ensure node exists
            self._target_ij = (ni, nj)
            self._target_dir = next_dir
            self._traversal_start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                f'Planning: move {next_dir} from {self._current_ij} to {self._target_ij}'
            )
            self._set_state(S_TRAVERSING)
        else:
            # No unvisited neighbours — try to backtrack
            backtrack = self._graph.backtrack_path(self._current_ij, self._travel_dir)
            if backtrack is None or len(backtrack) <= 1:
                # Entire reachable graph explored
                self.get_logger().info('All reachable nodes visited — mission complete.')
                self._set_state(S_DONE)
            else:
                self._backtrack_waypoints = backtrack
                self._backtrack_wp_idx = 1  # index 0 is current position
                self.get_logger().info(
                    f'Backtracking via {len(backtrack)-1} waypoints to find unvisited branch.'
                )
                self._set_state(S_BACKTRACKING)

    def _bfs_next_direction(self) -> Optional[str]:
        """
        BFS-flavoured next step: pick the direction toward the nearest
        unvisited node by shortest hop count.
        """
        backtrack = self._graph.backtrack_path(self._current_ij, self._travel_dir)
        if backtrack is None or len(backtrack) < 2:
            return None
        # The second element is the first step
        next_ij = backtrack[1]
        for d in DIRECTIONS:
            ni, nj = self._graph.neighbor_ij(*self._current_ij, d)
            if (ni, nj) == next_ij:
                return d
        return None

    # ======================================================================
    # State: TRAVERSING
    # ======================================================================

    def _state_traversing(self):
        if not self._data_ready() or self._target_ij is None or self._target_dir is None:
            self._publish_stop_path()
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Check timeout
        if (self._traversal_start_time is not None and
                now - self._traversal_start_time > self._traversal_timeout_sec):
            self._stuck_count += 1
            self.get_logger().warn(
                f'Traversal timeout toward {self._target_ij} '
                f'(stuck_count={self._stuck_count})'
            )
            if self._stuck_count >= self._max_stuck:
                self.get_logger().error(
                    'Exceeded max stuck count — attempting recovery backup.'
                )
                self._publish_recovery_backup()
                self._stuck_count = 0
                # Stay in TRAVERSING but reset timer so we try again
                self._traversal_start_time = now + 2.0  # pause 2 s
            else:
                # Try a micro recovery and retry
                self._publish_recovery_backup()
                self._traversal_start_time = now + 1.5
            return

        tx, ty = self._node_center_xy(self._target_ij)
        dist = math.hypot(self._odom_x - tx, self._odom_y - ty)

        # Arrived?
        if dist <= self._center_tol * 2.0:
            self._stuck_count = 0
            prev_ij = self._current_ij
            self._current_ij = self._target_ij
            self._travel_dir = self._target_dir
            if prev_ij is not None:
                self._graph.mark_edge_traversed(prev_ij, self._current_ij)
            self._graph.mark_visited(*self._current_ij)
            self.get_logger().info(
                f'Arrived at node {self._current_ij} (dist={dist:.3f} m)'
            )
            self._target_ij = None
            self._target_dir = None
            self._publish_markers()
            self._set_state(S_AT_NODE)
            return

        # Still travelling — publish path at regular intervals
        if self._should_refresh_path():
            heading_yaw = self._map_yaw_for_direction(self._target_dir)
            self._publish_path_to(tx, ty, heading_yaw)

    # ======================================================================
    # State: BACKTRACKING
    # ======================================================================

    def _state_backtracking(self):
        if not self._data_ready():
            self._publish_stop_path()
            return

        if not self._backtrack_waypoints or self._backtrack_wp_idx >= len(self._backtrack_waypoints):
            # Finished backtracking
            self._set_state(S_AT_NODE)
            return

        target_ij = self._backtrack_waypoints[self._backtrack_wp_idx]
        tx, ty = self._node_center_xy(target_ij)
        dist = math.hypot(self._odom_x - tx, self._odom_y - ty)

        if dist <= self._center_tol * 2.0:
            self._current_ij = target_ij
            # Determine travel direction from previous waypoint
            prev_ij = self._backtrack_waypoints[self._backtrack_wp_idx - 1]
            di = target_ij[0] - prev_ij[0]
            dj = target_ij[1] - prev_ij[1]
            for d, (ddi, ddj) in DIRECTION_DELTA.items():
                if (ddi, ddj) == (di, dj):
                    self._travel_dir = d
                    break
            self._backtrack_wp_idx += 1
            if self._backtrack_wp_idx >= len(self._backtrack_waypoints):
                self.get_logger().info(f'Backtrack complete at {self._current_ij}')
                self._set_state(S_AT_NODE)
            return

        # Navigate to current waypoint
        if self._should_refresh_path():
            # Infer heading from direction of travel
            prev_ij = self._backtrack_waypoints[self._backtrack_wp_idx - 1]
            di = target_ij[0] - prev_ij[0]
            dj = target_ij[1] - prev_ij[1]
            heading_yaw = math.atan2(float(dj), float(di))  # approximate
            for d, (ddi, ddj) in DIRECTION_DELTA.items():
                if (ddi, ddj) == (di, dj):
                    heading_yaw = self._map_yaw_for_direction(d)
                    break
            self._publish_path_to(tx, ty, heading_yaw)

    # ======================================================================
    # State: EXIT
    # ======================================================================

    def _state_exit(self):
        self._publish_stop_path()
        self.get_logger().info('Saving graph and stopping — exit found.')
        self._save_graph()
        self._publish_markers()
        self._set_state(S_DONE)

    # ======================================================================
    # State: DONE
    # ======================================================================

    def _state_done(self):
        # Keep publishing stop path so the robot stays still
        if self._should_refresh_path():
            self._publish_stop_path()

    # ======================================================================
    # Exit detection
    # ======================================================================

    def _is_exit(self, clearances: Dict[str, float]) -> bool:
        """True if ≥2 cardinal directions have clearance > exit_threshold_m."""
        n_open = sum(1 for c in clearances.values() if c >= self._exit_thresh)
        return n_open >= 2

    # ======================================================================
    # Recovery backup
    # ======================================================================

    def _publish_recovery_backup(self):
        """Publish a short backward path to escape a wall."""
        # Go backward (opposite of current travel direction)
        if self._travel_dir is not None:
            back_yaw = self._map_yaw_for_direction(OPPOSITE[self._travel_dir])
        else:
            back_yaw = normalize_angle(self._odom_yaw + math.pi)
        tx = self._odom_x + self._recovery_m * math.cos(back_yaw)
        ty = self._odom_y + self._recovery_m * math.sin(back_yaw)
        self._publish_path_to(tx, ty, back_yaw)
        self.get_logger().info('Published recovery backup path.')

    # ======================================================================
    # Graph save
    # ======================================================================

    def _save_graph(self):
        if self._graph is None:
            return
        try:
            save_graph_json(self._graph, self._graph_save_path)
            self.get_logger().info(f'Graph saved to {self._graph_save_path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to save graph: {exc}')

    # ======================================================================
    # RViz markers
    # ======================================================================

    def _publish_markers(self):
        if self._graph is None:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # --- Delete all old markers first ---
        del_marker = Marker()
        del_marker.header.stamp = now
        del_marker.header.frame_id = self._odom_frame
        del_marker.action = Marker.DELETEALL
        ma.markers.append(del_marker)

        node_id = 0
        for node in self._graph.all_nodes():
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self._odom_frame
            m.ns = NS_NODE
            m.id = node_id
            node_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = node.x_m
            m.pose.position.y = node.y_m
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.12
            m.scale.y = 0.12
            m.scale.z = 0.12
            r, g, b, a = NODE_COLORS.get(node.node_type, (0.5, 0.5, 0.5, 0.8))
            # Dim backtracked nodes
            if node.visit_state == 'backtracked':
                a = 0.4
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            ma.markers.append(m)

            # Label (node type + visit order)
            lm = Marker()
            lm.header.stamp = now
            lm.header.frame_id = self._odom_frame
            lm.ns = NS_NODE + '_label'
            lm.id = node_id
            node_id += 1
            lm.type = Marker.TEXT_VIEW_FACING
            lm.action = Marker.ADD
            lm.pose.position.x = node.x_m
            lm.pose.position.y = node.y_m
            lm.pose.position.z = 0.18
            lm.pose.orientation.w = 1.0
            lm.scale.z = 0.08
            lm.color.r = 1.0
            lm.color.g = 1.0
            lm.color.b = 1.0
            lm.color.a = 1.0
            lm.text = f'({node.grid_i},{node.grid_j})\n#{node.visit_order}'
            ma.markers.append(lm)

        edge_id = 5000
        for edge in self._graph.all_edges():
            n_from = self._graph.get_node(*edge.from_ij)
            n_to = self._graph.get_node(*edge.to_ij)
            if n_from is None or n_to is None:
                continue
            em = Marker()
            em.header.stamp = now
            em.header.frame_id = self._odom_frame
            em.ns = NS_EDGE
            em.id = edge_id
            edge_id += 1
            em.type = Marker.ARROW
            em.action = Marker.ADD
            em.scale.x = 0.03   # shaft diameter
            em.scale.y = 0.06   # head diameter
            em.scale.z = 0.0
            em.color.r = 0.8 if edge.traversed else 0.4
            em.color.g = 0.8 if edge.traversed else 0.4
            em.color.b = 0.8 if edge.traversed else 0.4
            em.color.a = 1.0
            from_pt = PoseStamped()
            from_pt.pose.position.x = n_from.x_m
            from_pt.pose.position.y = n_from.y_m
            to_pt = PoseStamped()
            to_pt.pose.position.x = n_to.x_m
            to_pt.pose.position.y = n_to.y_m
            # ARROW type with points
            p1 = Point()
            p1.x = n_from.x_m
            p1.y = n_from.y_m
            p1.z = 0.03
            p2 = Point()
            p2.x = n_to.x_m
            p2.y = n_to.y_m
            p2.z = 0.03
            em.points.append(p1)
            em.points.append(p2)
            ma.markers.append(em)

        # Current target marker
        if self._target_ij is not None:
            tx, ty = self._node_center_xy(self._target_ij)
            tm = Marker()
            tm.header.stamp = now
            tm.header.frame_id = self._odom_frame
            tm.ns = NS_TARGET
            tm.id = 9999
            tm.type = Marker.CYLINDER
            tm.action = Marker.ADD
            tm.pose.position.x = tx
            tm.pose.position.y = ty
            tm.pose.position.z = 0.0
            tm.pose.orientation.w = 1.0
            tm.scale.x = 0.18
            tm.scale.y = 0.18
            tm.scale.z = 0.08
            tm.color.r = 1.0
            tm.color.g = 1.0
            tm.color.b = 0.0
            tm.color.a = 0.9
            ma.markers.append(tm)

        self._marker_pub.publish(ma)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MazeMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._graph is not None:
            node._save_graph()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
