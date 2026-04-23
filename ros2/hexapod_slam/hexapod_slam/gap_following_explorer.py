#!/usr/bin/env python3

"""
gap_following_explorer.py

High-level purpose:
    Reactive local exploration node that uses the current LiDAR scan to choose an open heading,
    then publishes a short rolling path in front of the robot. This node is intended for
    short-horizon obstacle avoidance and local movement, not long-term maze memory.

High-level behavior:
    - Subscribes to LaserScan and Odometry.
    - Scores candidate headings based on obstacle clearance, gap width, and optional forward bias.
    - Chooses a heading in the robot base frame.
    - Publishes a short Path toward that heading.
    - Publishes a stop/decision point when the robot reaches a wall or must replan.
    - Can be used as a local follower beneath a higher-level maze graph planner.

Inputs:
    - /scan (sensor_msgs/msg/LaserScan)
    - /odom (nav_msgs/msg/Odometry)

Outputs:
    - Path message for local motion
    - PointStamped decision marker when replanning is required

Publishes:
    - path_topic (default: "path") -> nav_msgs/msg/Path
    - stop_point_topic (default: "decision_point") -> geometry_msgs/msg/PointStamped

Subscribes:
    - scan_topic (default: "/scan") -> sensor_msgs/msg/LaserScan
    - odom_topic (default: "odom") -> nav_msgs/msg/Odometry

Key parameters:
    - scan_topic: "/scan"
    - odom_topic: "odom"
    - path_topic: "path"
    - stop_point_topic: "decision_point"
    - control_rate_hz: 5.0
    - path_publish_period_sec: 0.5
    - decision_pause_sec: 0.75
    - scan_timeout_sec: 1.0
    - odom_timeout_sec: 1.0

    Local navigation thresholds:
    - stop_distance_m: 0.60
        Minimum clearance before stopping and replanning.
    - open_distance_m: 0.72
        Preferred clearance threshold used when scoring open gaps.
    - goal_backoff_m: 0.45
        Distance to remain behind a detected obstacle.
    - max_goal_distance_m: 1.25
        Maximum forward goal distance.
    - min_goal_distance_m: 0.20
        Minimum forward goal distance when space is tight.
    - footprint_radius_m: 0.30
        Assumed circular robot radius used to keep the body away from walls.
    - wall_clearance_margin_m: 0.10
        Extra buffer added beyond the robot radius when evaluating wall clearance.

    Candidate scoring parameters:
    - clearance_window_deg: 15.0
        Half-window around a candidate heading used to evaluate minimum clearance.
    - forward_bias_weight: 1.5
        Positive values prefer forward-facing headings.
    - gap_width_weight: 0.20
        Reward for wider openings.
    - reverse_penalty_weight: 1.50
        Penalty for selecting the direction the robot just came from.
    - min_gap_width_deg: 18.0
        Minimum angular gap width considered traversable.
    - reverse_avoidance_deg: 70.0
        Avoid selecting headings too close to the reverse direction.
    - candidate_sample_count: 121
        Number of scan samples evaluated during heading selection.
    - max_replan_attempts: 8
        Number of consecutive failed replans before declaring mission complete.

    Corner-handling parameters:
    - corner_mode_enabled: True
        Detect "front blocked, one side open" turns and use a corner-specific heading policy.
    - corner_forward_blocked_distance_m: 0.72
        Front clearance below which the robot treats the current view as a corner candidate.
    - corner_side_open_distance_m: 0.82
        Side clearance required to treat left or right as the open turn direction.
    - corner_clearance_window_deg: 8.0
        Narrower window used to evaluate headings while turning a corner.
    - corner_goal_distance_m: 0.28
        Shorter rolling-path distance used during corner mode.
    - corner_outer_wall_bias_m: 0.10
        Lateral offset that biases the corner target away from the inside wall.
    - corner_commit_time_sec: 1.0
        How long the corner-specific target shaping remains active after selecting a corner heading.

Notes:
    - This node is best used as a local obstacle-aware motion primitive.
    - For maze solving, pair it with a persistent graph planner that decides which branch to take.
    - The published Path is short-horizon and not a full global plan.
"""



import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def angular_distance(angle_a_rad: float, angle_b_rad: float) -> float:
    return normalize_angle(angle_a_rad - angle_b_rad)


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def reverse_alignment_score(
    angle_rad: float,
    incoming_angle_base_rad: Optional[float],
) -> float:
    """Positive values indicate the heading points back toward the incoming corridor."""
    if incoming_angle_base_rad is None:
        return 0.0
    return math.cos(angular_distance(angle_rad, incoming_angle_base_rad))


def is_backtracking_heading(
    angle_rad: float,
    incoming_angle_base_rad: Optional[float],
) -> bool:
    """True when the heading still lies in the half-plane behind the robot."""
    return reverse_alignment_score(angle_rad, incoming_angle_base_rad) > 1e-6


@dataclass(frozen=True)
class HeadingCandidate:
    angle_rad: float
    clearance_m: float
    gap_width_rad: float
    gap_width_m: float
    score: float
    score_clearance: float
    score_gap_bonus: float
    score_forward_bias: float
    score_reverse_penalty: float


class GapFollowingExplorerNode(Node):
    def __init__(self):
        super().__init__('gap_following_explorer')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_yaw_offset_rad', 0.0)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('path_topic', 'path')
        self.declare_parameter('stop_point_topic', 'decision_point')
        self.declare_parameter('control_rate_hz', 5.0)
        self.declare_parameter('path_publish_period_sec', 0.5)
        self.declare_parameter('decision_pause_sec', 0.75)
        self.declare_parameter('scan_timeout_sec', 1.0)
        self.declare_parameter('odom_timeout_sec', 1.0)
        self.declare_parameter('stop_distance_m', 0.60)
        self.declare_parameter('open_distance_m', 0.72)
        self.declare_parameter('goal_backoff_m', 0.45)
        self.declare_parameter('max_goal_distance_m', 1.25)
        self.declare_parameter('min_goal_distance_m', 0.20)
        self.declare_parameter('footprint_radius_m', 0.30)
        self.declare_parameter('wall_clearance_margin_m', 0.10)
        self.declare_parameter('clearance_window_deg', 15.0)
        self.declare_parameter('forward_bias_weight', 1.5)
        self.declare_parameter('gap_width_weight', 0.20)
        self.declare_parameter('reverse_penalty_weight', 1.50)
        self.declare_parameter('min_gap_width_deg', 18.0)
        self.declare_parameter('reverse_avoidance_deg', 70.0)
        self.declare_parameter('candidate_sample_count', 121)
        self.declare_parameter('corner_mode_enabled', True)
        self.declare_parameter('corner_forward_blocked_distance_m', 0.72)
        self.declare_parameter('corner_side_open_distance_m', 0.82)
        self.declare_parameter('corner_clearance_window_deg', 8.0)
        self.declare_parameter('corner_min_turn_angle_deg', 35.0)
        self.declare_parameter('corner_max_turn_angle_deg', 105.0)
        self.declare_parameter('corner_preferred_turn_angle_deg', 70.0)
        self.declare_parameter('corner_goal_distance_m', 0.28)
        self.declare_parameter('corner_outer_wall_bias_m', 0.10)
        self.declare_parameter('corner_commit_time_sec', 1.0)
        self.declare_parameter('max_replan_attempts', 8)
        self.declare_parameter('max_yaw_drift_deg', 30.0)
        self.declare_parameter('min_progress_m', 0.10)
        self.declare_parameter('recovery_backup_m', 0.20)
        self.declare_parameter('log_candidate_decisions', True)
        self.declare_parameter('decision_log_top_k', 3)
        self.declare_parameter('tile_size_m', 1.2192)
        self.declare_parameter('tile_origin_x_m', 0.0)
        self.declare_parameter('tile_origin_y_m', 0.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.scan_yaw_offset_rad = float(self.get_parameter('scan_yaw_offset_rad').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.stop_point_topic = str(self.get_parameter('stop_point_topic').value)
        self.control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.path_publish_period_sec = max(0.1, float(self.get_parameter('path_publish_period_sec').value))
        self.decision_pause_sec = max(0.0, float(self.get_parameter('decision_pause_sec').value))
        self.scan_timeout_sec = max(0.1, float(self.get_parameter('scan_timeout_sec').value))
        self.odom_timeout_sec = max(0.1, float(self.get_parameter('odom_timeout_sec').value))
        configured_stop_distance_m = max(0.05, float(self.get_parameter('stop_distance_m').value))
        configured_open_distance_m = max(
            configured_stop_distance_m,
            float(self.get_parameter('open_distance_m').value),
        )
        configured_goal_backoff_m = max(0.0, float(self.get_parameter('goal_backoff_m').value))
        self.max_goal_distance_m = max(0.05, float(self.get_parameter('max_goal_distance_m').value))
        self.min_goal_distance_m = max(0.0, float(self.get_parameter('min_goal_distance_m').value))
        self.footprint_radius_m = max(0.0, float(self.get_parameter('footprint_radius_m').value))
        self.wall_clearance_margin_m = max(
            0.0,
            float(self.get_parameter('wall_clearance_margin_m').value),
        )
        self.minimum_wall_clearance_m = self.footprint_radius_m + self.wall_clearance_margin_m
        self.minimum_gap_width_m = 2.0 * self.minimum_wall_clearance_m
        self.stop_distance_m = max(configured_stop_distance_m, self.minimum_wall_clearance_m)
        self.open_distance_m = max(configured_open_distance_m, self.stop_distance_m)
        self.goal_backoff_m = max(configured_goal_backoff_m, self.minimum_wall_clearance_m)
        self.clearance_window_rad = math.radians(
            max(1.0, float(self.get_parameter('clearance_window_deg').value))
        )
        self.forward_bias_weight = float(self.get_parameter('forward_bias_weight').value)
        self.gap_width_weight = float(self.get_parameter('gap_width_weight').value)
        self.reverse_penalty_weight = float(self.get_parameter('reverse_penalty_weight').value)
        self.min_gap_width_rad = math.radians(
            max(1.0, float(self.get_parameter('min_gap_width_deg').value))
        )
        self.reverse_avoidance_rad = math.radians(
            max(0.0, float(self.get_parameter('reverse_avoidance_deg').value))
        )
        self.candidate_sample_count = max(12, int(self.get_parameter('candidate_sample_count').value))
        self.corner_mode_enabled = bool(self.get_parameter('corner_mode_enabled').value)
        self.corner_forward_blocked_distance_m = max(
            self.stop_distance_m,
            float(self.get_parameter('corner_forward_blocked_distance_m').value),
        )
        self.corner_side_open_distance_m = max(
            self.open_distance_m,
            float(self.get_parameter('corner_side_open_distance_m').value),
        )
        self.corner_clearance_window_rad = math.radians(
            max(1.0, float(self.get_parameter('corner_clearance_window_deg').value))
        )
        corner_min_turn_angle_deg = max(
            5.0,
            float(self.get_parameter('corner_min_turn_angle_deg').value),
        )
        corner_max_turn_angle_deg = max(
            corner_min_turn_angle_deg,
            float(self.get_parameter('corner_max_turn_angle_deg').value),
        )
        self.corner_min_turn_angle_rad = math.radians(corner_min_turn_angle_deg)
        self.corner_max_turn_angle_rad = math.radians(corner_max_turn_angle_deg)
        corner_preferred_turn_angle_deg = max(
            corner_min_turn_angle_deg,
            min(
                corner_max_turn_angle_deg,
                float(self.get_parameter('corner_preferred_turn_angle_deg').value),
            ),
        )
        self.corner_preferred_turn_angle_rad = math.radians(corner_preferred_turn_angle_deg)
        self.corner_goal_distance_m = max(
            0.05,
            float(self.get_parameter('corner_goal_distance_m').value),
        )
        self.corner_outer_wall_bias_m = max(
            0.0,
            float(self.get_parameter('corner_outer_wall_bias_m').value),
        )
        self.corner_commit_time_sec = max(
            0.0,
            float(self.get_parameter('corner_commit_time_sec').value),
        )
        self.max_replan_attempts = max(1, int(self.get_parameter('max_replan_attempts').value))
        self.max_yaw_drift_rad = math.radians(
            max(5.0, float(self.get_parameter('max_yaw_drift_deg').value))
        )
        self.min_progress_m = max(0.01, float(self.get_parameter('min_progress_m').value))
        self.recovery_backup_m = max(0.05, float(self.get_parameter('recovery_backup_m').value))
        self.log_candidate_decisions = bool(self.get_parameter('log_candidate_decisions').value)
        self.decision_log_top_k = max(1, int(self.get_parameter('decision_log_top_k').value))
        self.tile_size_m = max(0.1, float(self.get_parameter('tile_size_m').value))
        self.tile_origin_x_m = float(self.get_parameter('tile_origin_x_m').value)
        self.tile_origin_y_m = float(self.get_parameter('tile_origin_y_m').value)

        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)
        self.stop_point_publisher = self.create_publisher(PointStamped, self.stop_point_topic, 10)

        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.latest_scan: Optional[LaserScan] = None
        self._previous_scan: Optional[LaserScan] = None
        self.latest_scan_received_at = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_received_at = None
        self.odom_frame_id = 'odom'
        self.odom_x_m = 0.0
        self.odom_y_m = 0.0
        self.odom_yaw_rad = 0.0

        self.active_heading_world_rad: Optional[float] = None
        self.last_committed_heading_world_rad: Optional[float] = None
        self.corner_active_until = self.get_clock().now()
        self.corner_turn_sign = 0.0
        self.replan_ready_time = self.get_clock().now()
        self.last_path_publish_time = None
        self.last_data_warn_time_sec = -1.0

        # Position and yaw recorded when a heading is committed.
        # Used to measure actual travel and detect unexpected yaw drift.
        self.heading_committed_x_m = 0.0
        self.heading_committed_y_m = 0.0
        self.heading_committed_yaw_rad = 0.0

        # Dead-end detection: counts consecutive stops where the robot made
        # no meaningful progress.  Resets only when meaningful travel occurred.
        self.replan_attempts = 0
        self.mission_complete = False
        self._last_reported_tile = None

        self.get_logger().info('Gap-following explorer ready')
        self.get_logger().info(
            f'Listening to {self.scan_topic} and {self.odom_topic}, publishing path on {self.path_topic} '
            f'and stop points on {self.stop_point_topic}'
        )
        self.get_logger().info(
            f'Heading interpretation uses scan yaw offset {math.degrees(self.scan_yaw_offset_rad):.1f} deg '
            f'(laser -> base_link).'
        )
        self.get_logger().info(
            f'Footprint radius {self.footprint_radius_m:.2f} m with {self.wall_clearance_margin_m:.2f} m '
            f'wall buffer -> stop_distance={self.stop_distance_m:.2f} m, '
            f'goal_backoff={self.goal_backoff_m:.2f} m, min_gap_width={self.minimum_gap_width_m:.2f} m'
        )
        self.get_logger().info(
            f'Approx tile debug uses tile_size={self.tile_size_m:.4f} m with origin at '
            f'({self.tile_origin_x_m:.2f}, {self.tile_origin_y_m:.2f}) in odom.'
        )
        if self.corner_mode_enabled:
            self.get_logger().info(
                f'Corner mode enabled: front_blocked={self.corner_forward_blocked_distance_m:.2f} m, '
                f'side_open={self.corner_side_open_distance_m:.2f} m, '
                f'goal={self.corner_goal_distance_m:.2f} m, '
                f'bias={self.corner_outer_wall_bias_m:.2f} m.'
            )

    def scan_callback(self, msg: LaserScan):
        self._previous_scan = self.latest_scan
        self.latest_scan = msg
        self.latest_scan_received_at = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        self.latest_odom_received_at = self.get_clock().now()
        self.odom_frame_id = msg.header.frame_id or 'odom'
        self.odom_x_m = msg.pose.pose.position.x
        self.odom_y_m = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.odom_yaw_rad = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.maybe_log_tile_transition()

    def control_loop(self):
        # Once a dead end is declared stop publishing anything; robot is done.
        if self.mission_complete:
            return

        if not self.data_is_ready():
            # Sensor data went stale while the robot was moving — stop it
            # so it does not coast blindly into a wall.
            if self.active_heading_world_rad is not None:
                self.publish_stop_path()
            return

        now = self.get_clock().now()
        if now < self.replan_ready_time:
            return

        if self.active_heading_world_rad is None:
            selection_mode = 'forward_only'
            selected = None
            corner_selected = self.select_corner_heading()
            if corner_selected is not None:
                selected = corner_selected
                selection_mode = 'corner_mode'
            else:
                self.reset_corner_mode()
                selected = self.select_heading(allow_reverse=False)
                if selected is None:
                    self.get_logger().info(
                        f'No forward-only heading survived the current thresholds at '
                        f'{self.format_tile_estimate()} — retrying with reverse headings allowed.'
                    )
                    selection_mode = 'allow_reverse'
                    selected = self.select_heading(allow_reverse=True)
            if selected is None:
                # No traversable gap exists.  If we have already tried enough
                # times the robot has reached a true dead end.
                if self.replan_attempts >= self.max_replan_attempts:
                    self.mission_complete = True
                    self.get_logger().info(
                        f'Dead end reached after {self.replan_attempts} obstacle stop(s) at '
                        f'{self.format_tile_estimate()} — '
                        'no traversable gap found in any direction. Mission complete.'
                    )
                self.publish_stop_path()
                return

            # Commit the new heading and record position/yaw for progress tracking.
            self.active_heading_world_rad = normalize_angle(self.odom_yaw_rad + selected.angle_rad)
            self.heading_committed_x_m = self.odom_x_m
            self.heading_committed_y_m = self.odom_y_m
            self.heading_committed_yaw_rad = self.odom_yaw_rad
            self.get_logger().info(
                f'Heading selected ({selection_mode}) at {self.format_tile_estimate()}: '
                f'{math.degrees(selected.angle_rad):.1f} deg in base frame, '
                f'clearance {selected.clearance_m:.2f} m, gap width {math.degrees(selected.gap_width_rad):.1f} deg, '
                f'score {selected.score:.2f}'
            )
            self.publish_active_path(selected.clearance_m)
            return

        # If the robot has drifted significantly from its committed heading
        # (hexapod gait yaw drift), the forward clearance check is now looking
        # at the wrong physical direction.  Force a penalty-free replan so a
        # fresh heading is selected from the current orientation without
        # incrementing the stuck counter.
        yaw_drift = abs(normalize_angle(self.odom_yaw_rad - self.heading_committed_yaw_rad))
        if yaw_drift > self.max_yaw_drift_rad:
            self.get_logger().info(
                f'Yaw drift {math.degrees(yaw_drift):.1f} deg exceeds '
                f'{math.degrees(self.max_yaw_drift_rad):.0f} deg at {self.format_tile_estimate()} — '
                'replanning (no penalty).'
            )
            self.last_committed_heading_world_rad = self.active_heading_world_rad
            self.active_heading_world_rad = None
            self.publish_stop_path()
            self.replan_ready_time = self.get_clock().now() + Duration(
                seconds=self.decision_pause_sec
            )
            return

        active_clearance_window_rad = (
            self.corner_clearance_window_rad if self.corner_mode_is_active() else None
        )
        forward_clearance = self.clearance_for_world_heading(
            self.active_heading_world_rad,
            use_stable=True,
            window_rad=active_clearance_window_rad,
        )
        if forward_clearance <= self.stop_distance_m:
            self.handle_replan_stop(forward_clearance)
            return

        if self.should_refresh_path(now):
            self.publish_active_path(forward_clearance)

    def data_is_ready(self) -> bool:
        now = self.get_clock().now()
        scan_is_fresh = (
            self.latest_scan is not None
            and self.latest_scan_received_at is not None
            and (now - self.latest_scan_received_at).nanoseconds / 1e9 <= self.scan_timeout_sec
        )
        odom_is_fresh = (
            self.latest_odom is not None
            and self.latest_odom_received_at is not None
            and (now - self.latest_odom_received_at).nanoseconds / 1e9 <= self.odom_timeout_sec
        )

        if scan_is_fresh and odom_is_fresh:
            return True

        now_sec = now.nanoseconds / 1e9
        if now_sec - self.last_data_warn_time_sec > 2.0:
            missing = []
            if not scan_is_fresh:
                missing.append('scan')
            if not odom_is_fresh:
                missing.append('odom')
            self.get_logger().warn(
                f'Explorer is waiting for fresh {" and ".join(missing)} data before publishing paths'
            )
            self.last_data_warn_time_sec = now_sec
        return False

    def should_refresh_path(self, now) -> bool:
        if self.last_path_publish_time is None:
            return True
        return (now - self.last_path_publish_time).nanoseconds / 1e9 >= self.path_publish_period_sec

    def handle_replan_stop(self, clearance_m: float):
        dist_traveled = math.hypot(
            self.odom_x_m - self.heading_committed_x_m,
            self.odom_y_m - self.heading_committed_y_m,
        )
        if dist_traveled >= self.min_progress_m:
            # Robot made real progress before stopping — fresh start on the counter.
            self.replan_attempts = 0
        else:
            # Blocked without meaningful travel — count as a failed attempt.
            self.replan_attempts += 1
        self.get_logger().info(
            f'Obstacle at {clearance_m:.2f} m in {self.format_tile_estimate()} '
            f'(traveled {dist_traveled:.2f} m) — '
            f'stop #{self.replan_attempts}. Searching for a new gap.'
        )
        self.last_committed_heading_world_rad = self.active_heading_world_rad
        self.active_heading_world_rad = None
        self.reset_corner_mode()
        self.publish_stop_point()

        if self.replan_attempts >= self.max_replan_attempts:
            # Stuck against a wall — back up to create clearance, then replan.
            self.get_logger().info(
                f'Stuck after {self.replan_attempts} consecutive low-travel stops — '
                'executing recovery backup.'
            )
            self._publish_recovery_backup()
            self.replan_attempts = 0
            self.replan_ready_time = self.get_clock().now() + Duration(seconds=2.0)
        else:
            self.publish_stop_path()
            self.replan_ready_time = self.get_clock().now() + Duration(
                seconds=self.decision_pause_sec
            )

    def select_heading(self, allow_reverse: bool) -> Optional[HeadingCandidate]:
        if self.latest_scan is None:
            return None

        scan = self.latest_scan
        if len(scan.ranges) == 0 or abs(scan.angle_increment) < 1e-9:
            return None

        angle_increment = abs(scan.angle_increment)
        window_beams = max(1, int(self.clearance_window_rad / angle_increment))
        min_gap_beams = max(1, int(self.min_gap_width_rad / angle_increment))
        sample_stride = max(1, len(scan.ranges) // self.candidate_sample_count)
        open_threshold_m = self.open_distance_m if not allow_reverse else self.stop_distance_m

        incoming_angle_base_rad = self.incoming_angle_base_frame()
        best_candidate = None
        candidates = []
        for index in range(window_beams, len(scan.ranges) - window_beams, sample_stride):
            clearance_m = self.stable_window_clearance(index, window_beams)
            if clearance_m is None or clearance_m < open_threshold_m:
                continue

            gap_width_beams = self.gap_width_beams(index, open_threshold_m)
            if gap_width_beams < min_gap_beams:
                continue

            angle_rad = scan.angle_min + index * scan.angle_increment
            angle_rad = normalize_angle(angle_rad + self.scan_yaw_offset_rad)
            reverse_alignment = reverse_alignment_score(angle_rad, incoming_angle_base_rad)
            if (
                not allow_reverse
                and incoming_angle_base_rad is not None
                and (
                    abs(angular_distance(angle_rad, incoming_angle_base_rad)) < self.reverse_avoidance_rad
                    or is_backtracking_heading(angle_rad, incoming_angle_base_rad)
                )
            ):
                continue

            gap_width_rad = gap_width_beams * angle_increment
            gap_width_m = self.gap_width_m(clearance_m, gap_width_rad)
            if gap_width_m < self.minimum_gap_width_m:
                continue

            # Cap score components so range-max blind spots (LiDAR returning inf
            # behind the robot's own legs or at specular/transparent surfaces)
            # do not overwhelm genuine corridors.  The robot cannot benefit from
            # clearance beyond what it will actually travel anyway.
            score_clearance = min(clearance_m, self.max_goal_distance_m + self.goal_backoff_m)
            score_gap = min(gap_width_rad, math.pi)
            score_gap_bonus = self.gap_width_weight * score_gap
            score_forward_bias = self.forward_bias_weight * math.cos(angle_rad)
            score = score_clearance
            score += score_gap_bonus
            score += score_forward_bias
            score_reverse_penalty = 0.0

            if incoming_angle_base_rad is not None:
                score_reverse_penalty = self.reverse_penalty_weight * max(0.0, reverse_alignment)
                score -= score_reverse_penalty

            candidate = HeadingCandidate(
                angle_rad=angle_rad,
                clearance_m=clearance_m,
                gap_width_rad=gap_width_rad,
                gap_width_m=gap_width_m,
                score=score,
                score_clearance=score_clearance,
                score_gap_bonus=score_gap_bonus,
                score_forward_bias=score_forward_bias,
                score_reverse_penalty=score_reverse_penalty,
            )
            candidates.append(candidate)
            if best_candidate is None or candidate.score > best_candidate.score:
                best_candidate = candidate

        self.log_heading_decision(allow_reverse, incoming_angle_base_rad, candidates, best_candidate)
        return best_candidate

    def select_corner_heading(self) -> Optional[HeadingCandidate]:
        if not self.corner_mode_enabled or self.latest_scan is None:
            return None

        front_clearance_m = self.scan_window_clearance(
            0.0,
            window_rad=self.corner_clearance_window_rad,
            use_stable=True,
        )
        left_clearance_m = self.scan_window_clearance(
            math.pi / 2.0,
            window_rad=self.corner_clearance_window_rad,
            use_stable=True,
        )
        right_clearance_m = self.scan_window_clearance(
            -math.pi / 2.0,
            window_rad=self.corner_clearance_window_rad,
            use_stable=True,
        )

        if front_clearance_m is None or left_clearance_m is None or right_clearance_m is None:
            return None

        if front_clearance_m > self.corner_forward_blocked_distance_m:
            return None

        turn_sign = 0.0
        if (
            left_clearance_m >= self.corner_side_open_distance_m
            and right_clearance_m < self.corner_side_open_distance_m
        ):
            turn_sign = 1.0
        elif (
            right_clearance_m >= self.corner_side_open_distance_m
            and left_clearance_m < self.corner_side_open_distance_m
        ):
            turn_sign = -1.0
        else:
            return None

        scan = self.latest_scan
        angle_increment = abs(scan.angle_increment)
        if angle_increment < 1e-9:
            return None
        window_beams = max(1, int(self.corner_clearance_window_rad / angle_increment))
        sample_stride = max(1, len(scan.ranges) // self.candidate_sample_count)

        best_candidate = None
        for index in range(window_beams, len(scan.ranges) - window_beams, sample_stride):
            angle_rad = scan.angle_min + index * scan.angle_increment
            angle_rad = normalize_angle(angle_rad + self.scan_yaw_offset_rad)

            if turn_sign > 0.0:
                if not (self.corner_min_turn_angle_rad <= angle_rad <= self.corner_max_turn_angle_rad):
                    continue
            else:
                if not (
                    -self.corner_max_turn_angle_rad <= angle_rad <= -self.corner_min_turn_angle_rad
                ):
                    continue

            clearance_m = self.stable_window_clearance(index, window_beams)
            if clearance_m is None or clearance_m < self.stop_distance_m:
                continue

            gap_width_beams = self.gap_width_beams(index, self.stop_distance_m)
            gap_width_rad = gap_width_beams * angle_increment
            gap_width_m = self.gap_width_m(clearance_m, gap_width_rad)
            if gap_width_m < (0.8 * self.minimum_gap_width_m):
                continue

            preferred_angle_rad = turn_sign * self.corner_preferred_turn_angle_rad
            angle_error_rad = abs(angular_distance(angle_rad, preferred_angle_rad))
            score_clearance = min(clearance_m, self.max_goal_distance_m)
            score_turn_preference = -0.8 * angle_error_rad
            score_gap_bonus = 0.15 * min(gap_width_rad, math.pi)
            score = score_clearance + score_turn_preference + score_gap_bonus

            candidate = HeadingCandidate(
                angle_rad=angle_rad,
                clearance_m=clearance_m,
                gap_width_rad=gap_width_rad,
                gap_width_m=gap_width_m,
                score=score,
                score_clearance=score_clearance,
                score_gap_bonus=score_gap_bonus,
                score_forward_bias=score_turn_preference,
                score_reverse_penalty=0.0,
            )
            if best_candidate is None or candidate.score > best_candidate.score:
                best_candidate = candidate

        if best_candidate is None:
            return None

        self.corner_turn_sign = turn_sign
        self.corner_active_until = self.get_clock().now() + Duration(
            seconds=self.corner_commit_time_sec
        )
        return best_candidate

    def incoming_angle_base_frame(self) -> Optional[float]:
        if self.last_committed_heading_world_rad is None:
            return None
        return normalize_angle(self.last_committed_heading_world_rad + math.pi - self.odom_yaw_rad)

    def corner_mode_is_active(self) -> bool:
        return self.corner_turn_sign != 0.0 and self.get_clock().now() < self.corner_active_until

    def reset_corner_mode(self):
        self.corner_turn_sign = 0.0
        self.corner_active_until = self.get_clock().now()

    def clearance_for_world_heading(
        self,
        heading_world_rad: float,
        use_stable: bool = False,
        window_rad: Optional[float] = None,
    ) -> float:
        target_angle_rad = normalize_angle(heading_world_rad - self.odom_yaw_rad)
        return self.clearance_for_base_angle(
            target_angle_rad,
            use_stable=use_stable,
            window_rad=window_rad,
        )

    def clearance_for_base_angle(
        self,
        target_angle_rad: float,
        use_stable: bool = False,
        window_rad: Optional[float] = None,
    ) -> float:
        clearance_m = self.scan_window_clearance(
            target_angle_rad,
            window_rad=window_rad,
            use_stable=use_stable,
        )
        return 0.0 if clearance_m is None else clearance_m

    def scan_window_clearance(
        self,
        target_angle_rad: float,
        window_rad: Optional[float] = None,
        use_stable: bool = False,
    ) -> Optional[float]:
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            return None

        scan = self.latest_scan
        angle_increment = abs(scan.angle_increment)
        if angle_increment < 1e-9:
            return None
        window_width_rad = self.clearance_window_rad if window_rad is None else window_rad
        window_beams = max(1, int(window_width_rad / angle_increment))
        target_scan_angle_rad = normalize_angle(target_angle_rad - self.scan_yaw_offset_rad)
        raw_index = (target_scan_angle_rad - scan.angle_min) / scan.angle_increment
        center_index = int(round(raw_index))
        center_index = max(window_beams, min(len(scan.ranges) - 1 - window_beams, center_index))
        if use_stable:
            return self.stable_window_clearance(center_index, window_beams)
        return self.window_clearance(center_index, window_beams)

    def window_clearance(self, center_index: int, window_beams: int) -> Optional[float]:
        if self.latest_scan is None:
            return None

        min_clearance = None
        for index in range(center_index - window_beams, center_index + window_beams + 1):
            range_value = self.valid_range(self.latest_scan, index)
            if range_value is None:
                continue
            if min_clearance is None or range_value < min_clearance:
                min_clearance = range_value
        return min_clearance

    def stable_window_clearance(self, center_index: int, window_beams: int) -> Optional[float]:
        """Minimum clearance across the current and previous scan.

        The hexapod's legs periodically sweep through the LiDAR scan plane.
        A single scan can show a direction as fully clear (legs retracted) while
        the next scan shows a close obstacle (legs extended).  Taking the minimum
        across two consecutive scans filters these transient artifacts so the
        heading selection only picks directions that are clear in BOTH snapshots.
        """
        current = self.window_clearance(center_index, window_beams)
        if self._previous_scan is None:
            return current

        prev_min = None
        for index in range(center_index - window_beams, center_index + window_beams + 1):
            if index < 0 or index >= len(self._previous_scan.ranges):
                continue
            range_value = self.valid_range(self._previous_scan, index)
            if range_value is None:
                continue
            if prev_min is None or range_value < prev_min:
                prev_min = range_value

        if current is None:
            return prev_min
        if prev_min is None:
            return current
        return min(current, prev_min)

    def gap_width_beams(self, center_index: int, threshold_m: float) -> int:
        if self.latest_scan is None:
            return 0

        total = 1
        for direction in (-1, 1):
            index = center_index + direction
            while 0 <= index < len(self.latest_scan.ranges):
                range_value = self.valid_range(self.latest_scan, index)
                if range_value is None or range_value < threshold_m:
                    break
                total += 1
                index += direction
        return total

    def gap_width_m(self, clearance_m: float, gap_width_rad: float) -> float:
        if clearance_m <= 0.0 or gap_width_rad <= 0.0:
            return 0.0
        return 2.0 * clearance_m * math.sin(gap_width_rad / 2.0)

    def valid_range(self, scan: LaserScan, index: int) -> Optional[float]:
        range_value = scan.ranges[index]
        if math.isnan(range_value):
            return None
        if math.isinf(range_value):
            return scan.range_max
        if range_value < scan.range_min:
            # Obstacle is closer than sensor's minimum range — treat as range_min
            # so it still registers as a near obstacle rather than being ignored.
            return scan.range_min
        if range_value > scan.range_max:
            return scan.range_max
        return range_value

    def publish_active_path(self, forward_clearance_m: float):
        if self.active_heading_world_rad is None:
            return

        goal_distance_m = min(
            self.max_goal_distance_m,
            max(0.0, forward_clearance_m - self.goal_backoff_m),
        )
        if goal_distance_m < self.min_goal_distance_m:
            goal_distance_m = min(
                self.min_goal_distance_m,
                max(0.0, forward_clearance_m - 0.05),
            )

        if self.corner_mode_is_active():
            goal_distance_m = min(goal_distance_m, self.corner_goal_distance_m)

        if goal_distance_m <= 0.0:
            self.handle_replan_stop(forward_clearance_m)
            return

        target_x_m = self.odom_x_m + goal_distance_m * math.cos(self.active_heading_world_rad)
        target_y_m = self.odom_y_m + goal_distance_m * math.sin(self.active_heading_world_rad)
        if self.corner_mode_is_active() and self.corner_outer_wall_bias_m > 0.0:
            lateral_left_x = -math.sin(self.active_heading_world_rad)
            lateral_left_y = math.cos(self.active_heading_world_rad)
            outward_sign = -self.corner_turn_sign
            target_x_m += outward_sign * self.corner_outer_wall_bias_m * lateral_left_x
            target_y_m += outward_sign * self.corner_outer_wall_bias_m * lateral_left_y

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.odom_frame_id
        path_msg.poses.append(self.make_pose(self.odom_x_m, self.odom_y_m, self.active_heading_world_rad))
        path_msg.poses.append(self.make_pose(target_x_m, target_y_m, self.active_heading_world_rad))
        self.path_publisher.publish(path_msg)
        self.last_path_publish_time = self.get_clock().now()

    def _publish_recovery_backup(self):
        """Publish a short path in the reverse of the last committed heading.

        This moves the robot away from the wall it is stuck against so that the
        next heading selection has a better view of the surroundings.
        """
        # Back up in the direction opposite to the last committed heading.
        reverse_heading = normalize_angle(
            (self.last_committed_heading_world_rad or self.odom_yaw_rad) + math.pi
        )
        target_x_m = self.odom_x_m + self.recovery_backup_m * math.cos(reverse_heading)
        target_y_m = self.odom_y_m + self.recovery_backup_m * math.sin(reverse_heading)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.odom_frame_id
        path_msg.poses.append(self.make_pose(self.odom_x_m, self.odom_y_m, reverse_heading))
        path_msg.poses.append(self.make_pose(target_x_m, target_y_m, reverse_heading))
        self.path_publisher.publish(path_msg)
        self.last_path_publish_time = self.get_clock().now()

    def publish_stop_path(self):
        self.reset_corner_mode()
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.odom_frame_id
        path_msg.poses.append(self.make_pose(self.odom_x_m, self.odom_y_m, self.odom_yaw_rad))
        self.path_publisher.publish(path_msg)
        self.last_path_publish_time = self.get_clock().now()

    def publish_stop_point(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.point.x = self.odom_x_m
        msg.point.y = self.odom_y_m
        msg.point.z = 0.0
        self.stop_point_publisher.publish(msg)

    def make_pose(self, x_value_m: float, y_value_m: float, yaw_rad: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.odom_frame_id
        pose.pose.position.x = x_value_m
        pose.pose.position.y = y_value_m
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return pose

    def current_tile_indices(self) -> tuple[int, int]:
        half_tile_m = 0.5 * self.tile_size_m
        tile_x = math.floor(
            (self.odom_x_m - self.tile_origin_x_m + half_tile_m) / self.tile_size_m
        )
        tile_y = math.floor(
            (self.odom_y_m - self.tile_origin_y_m + half_tile_m) / self.tile_size_m
        )
        return int(tile_x), int(tile_y)

    def format_tile_estimate(self) -> str:
        tile_x, tile_y = self.current_tile_indices()
        return f'tile ({tile_x}, {tile_y}), odom=({self.odom_x_m:.2f}, {self.odom_y_m:.2f})'

    def maybe_log_tile_transition(self):
        tile_indices = self.current_tile_indices()
        if tile_indices == self._last_reported_tile:
            return
        self._last_reported_tile = tile_indices
        self.get_logger().info(
            f'Approx maze tile update (odom-based): {self.format_tile_estimate()}'
        )

    def log_heading_decision(
        self,
        allow_reverse: bool,
        incoming_angle_base_rad: Optional[float],
        candidates,
        selected: Optional[HeadingCandidate],
    ):
        if not self.log_candidate_decisions:
            return

        pass_name = 'allow_reverse' if allow_reverse else 'forward_only'
        incoming_text = (
            'none'
            if incoming_angle_base_rad is None
            else f'{math.degrees(incoming_angle_base_rad):.1f} deg'
        )
        if not candidates:
            self.get_logger().info(
                f'Heading decision {pass_name} at {self.format_tile_estimate()}: '
                f'no candidates (incoming={incoming_text}).'
            )
            return

        top_candidates = sorted(candidates, key=lambda candidate: candidate.score, reverse=True)[
            :self.decision_log_top_k
        ]
        candidate_parts = []
        for index, candidate in enumerate(top_candidates, start=1):
            candidate_parts.append(
                f'#{index} angle={math.degrees(candidate.angle_rad):.1f} deg '
                f'clear={candidate.clearance_m:.2f} m '
                f'gap={math.degrees(candidate.gap_width_rad):.1f} deg/{candidate.gap_width_m:.2f} m '
                f'score={candidate.score:.2f} '
                f'[clear={candidate.score_clearance:.2f}, gap={candidate.score_gap_bonus:.2f}, '
                f'fwd={candidate.score_forward_bias:+.2f}, rev_penalty={candidate.score_reverse_penalty:.2f}]'
            )

        selected_text = (
            'none'
            if selected is None
            else f'{math.degrees(selected.angle_rad):.1f} deg (score {selected.score:.2f})'
        )
        self.get_logger().info(
            f'Heading decision {pass_name} at {self.format_tile_estimate()}: '
            f'incoming={incoming_text}, selected={selected_text}. '
            f'Top candidates: {" | ".join(candidate_parts)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowingExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
