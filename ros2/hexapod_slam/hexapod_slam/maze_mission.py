#!/usr/bin/env python3
"""
maze_mission.py

Hexapod Mission 1: Maze Traversal

Traverses a physical maze built from 2-ft × 2-ft tiles by maintaining a
grid-based topological graph, classifying walls at each node with temporal
filtering, and executing DFS/BFS traversal with left-first branch ordering.

Cardinal direction convention
─────────────────────────────
  'N' = the direction the robot is FACING when the mission starts.
  All other directions follow from that:
    'E' = 90° right of N   (robot's right at startup)
    'S' = 180° from N      (backward)
    'W' = 90° left of N    (robot's left at startup)

  _north_yaw records the odom-frame yaw at startup and is used to rotate
  all logical directions into the world frame.

State machine
─────────────
  WAIT_FOR_DATA → AT_NODE → CLASSIFYING → PLANNING
                                             ↓
                                         TRAVERSING ← → BACKTRACKING
                                             ↓
                                            EXIT → DONE

Topics published
────────────────
  maze_mission/path     nav_msgs/Path            — consumed by crab_path_follower
  maze_mission/markers  visualization_msgs/MarkerArray — RViz graph

Topics subscribed
─────────────────
  scan_topic  (default /scan)  sensor_msgs/LaserScan
  odom_topic  (default odom)   nav_msgs/Odometry
"""

from __future__ import annotations

import math
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from hexapod_slam.grid_graph import (
    DIRECTIONS,
    DIRECTION_DELTA,
    OPPOSITE,
    TILE_SIZE_M,
    UNVISITED,
    GridGraph,
    save_graph_json,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _normalise(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# ---------------------------------------------------------------------------
# State constants
# ---------------------------------------------------------------------------

S_WAIT_FOR_DATA = 'WAIT_FOR_DATA'
S_AT_NODE       = 'AT_NODE'
S_CLASSIFYING   = 'CLASSIFYING'
S_PLANNING      = 'PLANNING'
S_TRAVERSING    = 'TRAVERSING'
S_BACKTRACKING  = 'BACKTRACKING'
S_EXIT          = 'EXIT'
S_DONE          = 'DONE'

NS_NODE   = 'maze_nodes'
NS_EDGE   = 'maze_edges'
NS_TARGET = 'maze_target'

NODE_COLORS: Dict[str, Tuple[float, float, float, float]] = {
    'unknown':   (0.5, 0.5, 0.5, 0.8),
    'corridor':  (0.2, 0.8, 0.2, 0.9),
    'junction':  (0.2, 0.4, 1.0, 0.9),
    'dead_end':  (1.0, 0.4, 0.2, 0.9),
    'exit':      (1.0, 1.0, 0.0, 1.0),
}

# Thresholds derived from tile geometry.
# The tile wall is at TILE_SIZE_M/2 = 0.3048 m from the node centre.
# We use generous margins to tolerate ±10 cm positioning error.
_HALF_TILE      = TILE_SIZE_M / 2.0          # 0.3048 m — wall-to-centre distance
_WALL_THRESH    = _HALF_TILE + 0.10          # 0.4048 m — anything closer = wall
_OPEN_THRESH    = TILE_SIZE_M * 0.75         # 0.4572 m — anything further = open passage
_EXIT_THRESH    = TILE_SIZE_M * 2.0          # 1.2192 m — 2 open tiles → exit


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class MazeMissionNode(Node):

    def __init__(self):
        super().__init__('maze_mission')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('scan_topic',              '/scan')
        self.declare_parameter('odom_topic',              'odom')
        self.declare_parameter('wall_threshold_m',        _WALL_THRESH)
        self.declare_parameter('open_threshold_m',        _OPEN_THRESH)
        self.declare_parameter('exit_threshold_m',        _EXIT_THRESH)
        self.declare_parameter('centering_tolerance_m',   0.08)
        self.declare_parameter('classification_scans',    5)
        self.declare_parameter('classification_ratio',    0.8)
        self.declare_parameter('traversal_speed_mps',     0.04)
        self.declare_parameter('traversal_timeout_factor', 3.0)
        self.declare_parameter('path_publish_rate_hz',    10.0)
        self.declare_parameter('scan_window_deg',         15.0)
        self.declare_parameter('graph_save_path',         '/tmp/maze_graph.json')
        self.declare_parameter('use_bfs',                 False)
        self.declare_parameter('recovery_backup_m',       0.20)

        p = self.get_parameter
        self._scan_topic     = str(p('scan_topic').value)
        self._odom_topic     = str(p('odom_topic').value)
        self._wall_thresh    = float(p('wall_threshold_m').value)
        self._open_thresh    = float(p('open_threshold_m').value)
        self._exit_thresh    = float(p('exit_threshold_m').value)
        self._center_tol     = float(p('centering_tolerance_m').value)
        self._cls_scans      = max(1, int(p('classification_scans').value))
        self._cls_ratio      = float(p('classification_ratio').value)
        self._speed          = max(0.01, float(p('traversal_speed_mps').value))
        self._timeout_factor = float(p('traversal_timeout_factor').value)
        self._path_rate_hz   = float(p('path_publish_rate_hz').value)
        self._scan_win_rad   = math.radians(max(1.0, float(p('scan_window_deg').value)))
        self._graph_save     = str(p('graph_save_path').value)
        self._use_bfs        = bool(p('use_bfs').value)
        self._recovery_m     = float(p('recovery_backup_m').value)

        self._traversal_timeout = self._timeout_factor * (TILE_SIZE_M / self._speed)
        self._path_period       = 1.0 / max(1.0, self._path_rate_hz)

        # ------------------------------------------------------------------
        # Publishers / subscribers
        # ------------------------------------------------------------------
        self._path_pub   = self.create_publisher(Path,        'maze_mission/path',    10)
        self._marker_pub = self.create_publisher(MarkerArray, 'maze_mission/markers', 10)

        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._scan_cb, 10)
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_cb, 10)

        self._timer = self.create_timer(1.0 / 20.0, self._tick)  # 20 Hz

        # ------------------------------------------------------------------
        # Sensor state
        # ------------------------------------------------------------------
        self._scan: Optional[LaserScan] = None
        self._prev_scan: Optional[LaserScan] = None
        self._scan_time: Optional[float] = None

        self._odom_x     = 0.0
        self._odom_y     = 0.0
        self._odom_yaw   = 0.0
        self._odom_frame = 'odom'
        self._odom_time: Optional[float] = None

        # ------------------------------------------------------------------
        # Graph / navigation state
        # ------------------------------------------------------------------
        self._graph: Optional[GridGraph] = None
        self._current_ij: Optional[Tuple[int, int]] = None
        self._travel_dir: Optional[str] = None
        self._north_yaw: Optional[float] = None   # world yaw == logical 'N'

        # Classification buffer
        self._cls_buf: deque = deque(maxlen=self._cls_scans)
        self._cls_collected = 0

        # Traversal
        self._target_ij: Optional[Tuple[int, int]] = None
        self._target_dir: Optional[str] = None
        self._trav_start: Optional[float] = None
        self._last_path_t: Optional[float] = None

        # Backtrack
        self._bt_waypoints: List[Tuple[int, int]] = []
        self._bt_idx = 0

        # Stuck counter
        self._stuck = 0
        self._max_stuck = 5

        # ------------------------------------------------------------------
        # State machine
        # ------------------------------------------------------------------
        self._state = S_WAIT_FOR_DATA

        self.get_logger().info(
            f'MazeMission started ({"BFS" if self._use_bfs else "DFS left-first"}), '
            f'tile={TILE_SIZE_M:.4f} m | '
            f'wall<{self._wall_thresh:.3f} m, open>{self._open_thresh:.3f} m, '
            f'exit>{self._exit_thresh:.3f} m'
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
        self._odom_yaw  = _quat_to_yaw(o.x, o.y, o.z, o.w)
        self._odom_frame = msg.header.frame_id or 'odom'
        self._odom_time  = self.get_clock().now().nanoseconds / 1e9

    # ======================================================================
    # Tick
    # ======================================================================

    def _tick(self):
        {
            S_WAIT_FOR_DATA: self._st_wait,
            S_AT_NODE:       self._st_at_node,
            S_CLASSIFYING:   self._st_classifying,
            S_PLANNING:      self._st_planning,
            S_TRAVERSING:    self._st_traversing,
            S_BACKTRACKING:  self._st_backtracking,
            S_EXIT:          self._st_exit,
            S_DONE:          self._st_done,
        }.get(self._state, lambda: None)()

    def _set_state(self, s: str):
        self.get_logger().info(f'State: {self._state} → {s}')
        self._state = s

    # ======================================================================
    # Data-ready gate
    # ======================================================================

    def _data_ok(self) -> bool:
        now = self.get_clock().now().nanoseconds / 1e9
        return (
            self._scan is not None
            and self._scan_time is not None
            and (now - self._scan_time) < 2.0
            and self._odom_time is not None
            and (now - self._odom_time) < 2.0
        )

    # ======================================================================
    # Direction helpers  (all public-facing directions are LOGICAL)
    # ======================================================================

    def _world_yaw(self, logical_dir: str) -> float:
        """World-frame yaw for a logical cardinal direction.

        Uses _north_yaw so that logical 'N' always means the direction the
        robot was facing at startup, regardless of the odom frame orientation.
        """
        ny = self._north_yaw if self._north_yaw is not None else math.pi / 2.0
        offsets = {'N': 0.0, 'E': -math.pi / 2.0, 'S': math.pi, 'W': math.pi / 2.0}
        return _normalise(ny + offsets.get(logical_dir, 0.0))

    # ======================================================================
    # Clearance measurement
    # ======================================================================

    def _cardinal_clearance(self, logical_dir: str) -> float:
        """Min clearance in a logical direction (current + previous scan)."""
        if self._scan is None:
            return 0.0
        world_yaw  = self._world_yaw(logical_dir)
        base_angle = _normalise(world_yaw - self._odom_yaw)
        return self._clearance_at(base_angle)

    def _clearance_at(self, base_angle: float) -> float:
        c_cur  = self._scan_window(self._scan, base_angle)
        c_prev = self._scan_window(self._prev_scan, base_angle) if self._prev_scan else None
        vals = [v for v in (c_cur, c_prev) if v is not None]
        return min(vals) if vals else 0.0

    def _scan_window(self, scan: Optional[LaserScan], base_angle: float) -> Optional[float]:
        if scan is None or len(scan.ranges) == 0:
            return None
        inc = abs(scan.angle_increment)
        if inc < 1e-9:
            return None
        hw = max(1, int(self._scan_win_rad / inc))
        cx = int(round((base_angle - scan.angle_min) / scan.angle_increment))
        cx = max(hw, min(len(scan.ranges) - 1 - hw, cx))
        best = None
        for i in range(cx - hw, cx + hw + 1):
            if i < 0 or i >= len(scan.ranges):
                continue
            r = scan.ranges[i]
            if math.isnan(r):
                continue
            if math.isinf(r):
                r = scan.range_max
            elif r < scan.range_min:
                r = scan.range_min
            elif r > scan.range_max:
                r = scan.range_max
            if best is None or r < best:
                best = r
        return best

    def _all_clearances(self) -> Dict[str, float]:
        return {d: self._cardinal_clearance(d) for d in DIRECTIONS}

    # ======================================================================
    # Grid helpers
    # ======================================================================

    def _node_xy(self, ij: Tuple[int, int]) -> Tuple[float, float]:
        assert self._graph is not None
        return self._graph.ij_to_xy(*ij)

    def _dist_to(self, ij: Tuple[int, int]) -> float:
        tx, ty = self._node_xy(ij)
        return math.hypot(self._odom_x - tx, self._odom_y - ty)

    def _snap(self) -> Tuple[int, int]:
        assert self._graph is not None
        return self._graph.xy_to_ij(self._odom_x, self._odom_y)

    # ======================================================================
    # Path publishing
    # ======================================================================

    def _pub_path(self, tx: float, ty: float):
        yaw = math.atan2(ty - self._odom_y, tx - self._odom_x)
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.poses.append(self._pose(self._odom_x, self._odom_y, yaw))
        msg.poses.append(self._pose(tx, ty, yaw))
        self._path_pub.publish(msg)
        self._last_path_t = self.get_clock().now().nanoseconds / 1e9

    def _pub_stop(self):
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.poses.append(self._pose(self._odom_x, self._odom_y, self._odom_yaw))
        self._path_pub.publish(msg)
        self._last_path_t = self.get_clock().now().nanoseconds / 1e9

    def _need_refresh(self) -> bool:
        if self._last_path_t is None:
            return True
        return (self.get_clock().now().nanoseconds / 1e9 - self._last_path_t) >= self._path_period

    def _pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        p = PoseStamped()
        p.header.stamp    = self.get_clock().now().to_msg()
        p.header.frame_id = self._odom_frame
        p.pose.position.x = x
        p.pose.position.y = y
        _, _, qz, qw = _yaw_to_quat(yaw)
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    # ======================================================================
    # State: WAIT_FOR_DATA
    # ======================================================================

    def _st_wait(self):
        if not self._data_ok():
            return
        # Robot's startup facing direction becomes logical 'N'.
        self._north_yaw = self._odom_yaw
        self.get_logger().info(
            f'Sensor data ready — North = {math.degrees(self._north_yaw):.1f}° in odom.'
        )
        # Build grid with startup-heading-aligned coordinate system.
        self._graph = GridGraph(
            origin_x_m=self._odom_x,
            origin_y_m=self._odom_y,
            north_yaw=self._north_yaw,
        )
        self._current_ij = (0, 0)
        self._graph.mark_visited(0, 0)
        self._set_state(S_AT_NODE)

    # ======================================================================
    # State: AT_NODE
    # ======================================================================

    def _st_at_node(self):
        if not self._data_ok():
            self._pub_stop()
            return

        if self._current_ij is None:
            self._current_ij = self._snap()
            self._graph.mark_visited(*self._current_ij)

        # Quick exit check
        clrs = self._all_clearances()
        if self._is_exit(clrs):
            self._graph.set_exit(*self._current_ij)
            self.get_logger().info(
                f'EXIT detected at {self._current_ij}! '
                + ', '.join(f'{d}={v:.2f}m' for d, v in clrs.items())
            )
            self._set_state(S_EXIT)
            return

        self._cls_buf.clear()
        self._cls_collected = 0
        self._pub_stop()
        self._set_state(S_CLASSIFYING)

    # ======================================================================
    # State: CLASSIFYING
    # ======================================================================

    def _st_classifying(self):
        if not self._data_ok():
            self._pub_stop()
            return

        clrs = self._all_clearances()
        reading: Dict[str, bool] = {}
        for d, c in clrs.items():
            if c < self._wall_thresh:
                reading[d] = True    # wall
            elif c > self._open_thresh:
                reading[d] = False   # open

        self._cls_buf.append(reading)
        self._cls_collected += 1

        if self._cls_collected < self._cls_scans:
            self._pub_stop()
            return

        # Aggregate
        wall_map: Dict[str, bool] = {}
        for d in DIRECTIONS:
            w_votes = sum(1 for r in self._cls_buf if r.get(d) is True)
            o_votes = sum(1 for r in self._cls_buf if r.get(d) is False)
            n = w_votes + o_votes
            if n == 0:
                continue
            if w_votes / n >= self._cls_ratio:
                wall_map[d] = True
            elif o_votes / n >= self._cls_ratio:
                wall_map[d] = False

        if wall_map:
            self._graph.record_walls(*self._current_ij, wall_map)
            open_d = [d for d, v in wall_map.items() if not v]
            wall_d = [d for d, v in wall_map.items() if v]
            self.get_logger().info(
                f'Node {self._current_ij}: open={open_d} wall={wall_d} '
                f'type={self._graph.get_node(*self._current_ij).node_type}'
            )
        else:
            self.get_logger().warn(
                f'Node {self._current_ij}: no confident readings after '
                f'{self._cls_scans} scans — proceeding.'
            )

        # Register edges for open directions
        for d in self._graph.open_directions(*self._current_ij):
            ni, nj = self._graph.neighbor_ij(*self._current_ij, d)
            self._graph.add_edge(self._current_ij, (ni, nj), d)
            self._graph.add_edge((ni, nj), self._current_ij, OPPOSITE[d])

        self._publish_markers()
        self._set_state(S_PLANNING)

    # ======================================================================
    # State: PLANNING
    # ======================================================================

    def _st_planning(self):
        if self._current_ij is None:
            self._set_state(S_AT_NODE)
            return

        next_dir = (
            self._bfs_dir() if self._use_bfs
            else self._graph.dfs_next_direction(
                self._current_ij, self._travel_dir, allow_revisit=False)
        )

        if next_dir is not None:
            ni, nj = self._graph.neighbor_ij(*self._current_ij, next_dir)
            self._graph.get_or_create_node(ni, nj)
            self._target_ij  = (ni, nj)
            self._target_dir = next_dir
            self._trav_start = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                f'Move {next_dir}: {self._current_ij} → {self._target_ij}'
            )
            self._set_state(S_TRAVERSING)
        else:
            path = self._graph.backtrack_path(self._current_ij, self._travel_dir)
            if path is None or len(path) <= 1:
                self.get_logger().info('All nodes visited — mission complete.')
                self._set_state(S_DONE)
            else:
                self._bt_waypoints = path
                self._bt_idx = 1
                self.get_logger().info(
                    f'Backtracking {len(path)-1} hops to find unvisited branch.'
                )
                self._set_state(S_BACKTRACKING)

    def _bfs_dir(self) -> Optional[str]:
        path = self._graph.backtrack_path(self._current_ij, self._travel_dir)
        if path is None or len(path) < 2:
            return None
        next_ij = path[1]
        for d in DIRECTIONS:
            ni, nj = self._graph.neighbor_ij(*self._current_ij, d)
            if (ni, nj) == next_ij:
                return d
        return None

    # ======================================================================
    # State: TRAVERSING
    # ======================================================================

    def _st_traversing(self):
        if not self._data_ok() or self._target_ij is None or self._target_dir is None:
            self._pub_stop()
            return

        now = self.get_clock().now().nanoseconds / 1e9
        tx, ty = self._node_xy(self._target_ij)
        dist   = math.hypot(self._odom_x - tx, self._odom_y - ty)

        # ── Real-time obstacle check ─────────────────────────────────────────
        # Only abort if the clearance is both below the wall threshold AND
        # substantially closer than the remaining distance to the target.
        # This prevents false aborts when the robot legitimately approaches
        # the far wall of the destination tile (expected when getting close).
        fwd = self._cardinal_clearance(self._target_dir)
        if fwd < self._wall_thresh and fwd < dist * 0.8:
            self.get_logger().warn(
                f'Wall mid-traversal toward {self._target_ij}: '
                f'clearance={fwd:.2f} m, dist_to_target={dist:.2f} m — abort & replan.'
            )
            # Record the wall; do NOT touch the target node (it may not exist yet).
            self._graph.record_walls(*self._current_ij, {self._target_dir: True})
            self._pub_stop()
            self._stuck = 0
            self._target_ij  = None
            self._target_dir = None
            self._publish_markers()
            self._set_state(S_PLANNING)
            return

        # ── Arrival ──────────────────────────────────────────────────────────
        if dist <= self._center_tol * 2.0:
            self._stuck = 0
            prev = self._current_ij
            self._current_ij = self._target_ij
            self._travel_dir = self._target_dir
            if prev is not None:
                self._graph.mark_edge_traversed(prev, self._current_ij)
            self._graph.mark_visited(*self._current_ij)
            self.get_logger().info(f'Arrived {self._current_ij} (dist={dist:.3f} m)')
            self._target_ij  = None
            self._target_dir = None
            self._publish_markers()
            self._set_state(S_AT_NODE)
            return

        # ── Traversal timeout (mechanical jam, not a wall) ───────────────────
        if self._trav_start is not None and now - self._trav_start > self._traversal_timeout:
            self._stuck += 1
            self.get_logger().warn(
                f'Traversal timeout toward {self._target_ij} (stuck={self._stuck})'
            )
            if self._stuck >= self._max_stuck:
                self.get_logger().error('Max stuck count — recovery backup.')
                self._recovery_backup()
                self._stuck = 0
                self._trav_start = now + 2.0
            else:
                self._recovery_backup()
                self._trav_start = now + 1.5
            return

        # ── Keep publishing path ──────────────────────────────────────────────
        if self._need_refresh():
            self._pub_path(tx, ty)

    # ======================================================================
    # State: BACKTRACKING
    # ======================================================================

    def _st_backtracking(self):
        if not self._data_ok():
            self._pub_stop()
            return

        if not self._bt_waypoints or self._bt_idx >= len(self._bt_waypoints):
            self._set_state(S_AT_NODE)
            return

        target = self._bt_waypoints[self._bt_idx]
        tx, ty = self._node_xy(target)
        dist   = math.hypot(self._odom_x - tx, self._odom_y - ty)

        if dist <= self._center_tol * 2.0:
            self._current_ij = target
            prev = self._bt_waypoints[self._bt_idx - 1]
            di, dj = target[0] - prev[0], target[1] - prev[1]
            for d, (ddi, ddj) in DIRECTION_DELTA.items():
                if (ddi, ddj) == (di, dj):
                    self._travel_dir = d
                    break
            self._bt_idx += 1
            if self._bt_idx >= len(self._bt_waypoints):
                self.get_logger().info(f'Backtrack complete at {self._current_ij}')
                self._set_state(S_AT_NODE)
            return

        if self._need_refresh():
            self._pub_path(tx, ty)

    # ======================================================================
    # State: EXIT
    # ======================================================================

    def _st_exit(self):
        self._pub_stop()
        self.get_logger().info('Exit found — saving graph.')
        self._save_graph()
        self._publish_markers()
        self._set_state(S_DONE)

    # ======================================================================
    # State: DONE
    # ======================================================================

    def _st_done(self):
        if self._need_refresh():
            self._pub_stop()

    # ======================================================================
    # Exit detection
    # ======================================================================

    def _is_exit(self, clrs: Dict[str, float]) -> bool:
        return sum(1 for c in clrs.values() if c >= self._exit_thresh) >= 2

    # ======================================================================
    # Recovery
    # ======================================================================

    def _recovery_backup(self):
        if self._travel_dir is not None:
            back_yaw = self._world_yaw(OPPOSITE[self._travel_dir])
        else:
            back_yaw = _normalise(self._odom_yaw + math.pi)
        tx = self._odom_x + self._recovery_m * math.cos(back_yaw)
        ty = self._odom_y + self._recovery_m * math.sin(back_yaw)
        self._pub_path(tx, ty)
        self.get_logger().info('Recovery backup published.')

    # ======================================================================
    # Graph save
    # ======================================================================

    def _save_graph(self):
        if self._graph is None:
            return
        try:
            save_graph_json(self._graph, self._graph_save)
            self.get_logger().info(f'Graph saved → {self._graph_save}')
        except Exception as e:
            self.get_logger().error(f'Graph save failed: {e}')

    # ======================================================================
    # RViz markers
    # ======================================================================

    def _publish_markers(self):
        if self._graph is None:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        dm = Marker()
        dm.action = Marker.DELETEALL
        dm.header.stamp    = now
        dm.header.frame_id = self._odom_frame
        ma.markers.append(dm)

        uid = 0
        for node in self._graph.all_nodes():
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = self._odom_frame
            m.ns = NS_NODE; m.id = uid; uid += 1
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = node.x_m
            m.pose.position.y = node.y_m
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            r, g, b, a = NODE_COLORS.get(node.node_type, (0.5, 0.5, 0.5, 0.8))
            if node.visit_state == 'backtracked':
                a = 0.4
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a
            ma.markers.append(m)

            lm = Marker()
            lm.header.stamp    = now
            lm.header.frame_id = self._odom_frame
            lm.ns = NS_NODE + '_lbl'; lm.id = uid; uid += 1
            lm.type   = Marker.TEXT_VIEW_FACING
            lm.action = Marker.ADD
            lm.pose.position.x = node.x_m
            lm.pose.position.y = node.y_m
            lm.pose.position.z = 0.18
            lm.pose.orientation.w = 1.0
            lm.scale.z = 0.08
            lm.color.r = lm.color.g = lm.color.b = lm.color.a = 1.0
            lm.text = f'({node.grid_i},{node.grid_j})\n#{node.visit_order}'
            ma.markers.append(lm)

        for edge in self._graph.all_edges():
            nf = self._graph.get_node(*edge.from_ij)
            nt = self._graph.get_node(*edge.to_ij)
            if nf is None or nt is None:
                continue
            em = Marker()
            em.header.stamp    = now
            em.header.frame_id = self._odom_frame
            em.ns = NS_EDGE; em.id = uid; uid += 1
            em.type   = Marker.ARROW
            em.action = Marker.ADD
            em.scale.x = 0.03; em.scale.y = 0.06; em.scale.z = 0.0
            v = 0.8 if edge.traversed else 0.4
            em.color.r = em.color.g = em.color.b = v; em.color.a = 1.0
            p1 = Point(); p1.x = nf.x_m; p1.y = nf.y_m; p1.z = 0.03
            p2 = Point(); p2.x = nt.x_m; p2.y = nt.y_m; p2.z = 0.03
            em.points.append(p1); em.points.append(p2)
            ma.markers.append(em)

        if self._target_ij is not None:
            tx, ty = self._node_xy(self._target_ij)
            tm = Marker()
            tm.header.stamp    = now
            tm.header.frame_id = self._odom_frame
            tm.ns = NS_TARGET; tm.id = 9999
            tm.type   = Marker.CYLINDER
            tm.action = Marker.ADD
            tm.pose.position.x = tx; tm.pose.position.y = ty
            tm.pose.orientation.w = 1.0
            tm.scale.x = tm.scale.y = 0.18; tm.scale.z = 0.08
            tm.color.r = tm.color.g = 1.0; tm.color.b = 0.0; tm.color.a = 0.9
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
