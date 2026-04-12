#!/usr/bin/env python3
"""Crab-motion path follower for the hexapod robot.

Subscribes to a nav_msgs/Path (2-pose rolling path from the planner) and
converts it into geometry_msgs/Twist commands for crab (holonomic) motion.

Safety guarantees
─────────────────
1. Obstacle veto: subscribes to /scan and checks clearance in the commanded
   direction every tick.  If anything is within stop_distance_m the follower
   publishes zero velocity immediately, regardless of what the planner says.
2. Stale-data stop: if either the path OR the scan goes stale the follower
   publishes zero velocity.
3. Frame-correct velocity: the goal vector is always rotated from the odom
   (world) frame into the robot body frame using the current yaw from odom
   before being published as linear.x / linear.y.

Key design constraint: the robot body does not intentionally rotate, so
angular.z is always zero.  The world-to-body rotation accounts for any
residual yaw drift in the odometry.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CrabPathFollower(Node):
    def __init__(self):
        super().__init__('crab_path_follower')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('path_topic',          'path')
        self.declare_parameter('odom_topic',          'odom')
        self.declare_parameter('cmd_vel_topic',       'cmd_vel')
        self.declare_parameter('scan_topic',          '/scan')
        self.declare_parameter('constant_speed_mps',  0.04)
        self.declare_parameter('goal_tolerance_m',    0.05)
        # stop immediately if anything is this close in the commanded direction
        self.declare_parameter('stop_distance_m',     0.30)
        # path must be republished within this period or robot stops
        self.declare_parameter('path_timeout_sec',    0.30)
        # scan must be fresh within this period or robot stops
        self.declare_parameter('scan_timeout_sec',    0.50)
        self.declare_parameter('cmd_vel_rate_hz',     20.0)
        # half-angle of the scan window used for the obstacle check (degrees)
        self.declare_parameter('obstacle_window_deg', 15.0)

        path_topic      = str(self.get_parameter('path_topic').value)
        odom_topic      = str(self.get_parameter('odom_topic').value)
        cmd_vel_topic   = str(self.get_parameter('cmd_vel_topic').value)
        scan_topic      = str(self.get_parameter('scan_topic').value)
        self.speed      = max(0.001, float(self.get_parameter('constant_speed_mps').value))
        self.tolerance  = max(0.01,  float(self.get_parameter('goal_tolerance_m').value))
        self.stop_dist  = max(0.05,  float(self.get_parameter('stop_distance_m').value))
        self.path_timeout = max(0.05, float(self.get_parameter('path_timeout_sec').value))
        self.scan_timeout = max(0.05, float(self.get_parameter('scan_timeout_sec').value))
        rate_hz         = max(1.0,   float(self.get_parameter('cmd_vel_rate_hz').value))
        self._obstacle_window_rad = math.radians(
            max(1.0, float(self.get_parameter('obstacle_window_deg').value))
        )

        # ── State ────────────────────────────────────────────────────────────
        self.latest_path      = None
        self.latest_path_time = None

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.odom_time = None

        self.latest_scan      = None
        self.latest_scan_time = None

        # ── Publishers / Subscribers ─────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.create_subscription(Path,      path_topic,  self.path_callback, 10)
        self.create_subscription(Odometry,  odom_topic,  self.odom_callback, 10)
        self.create_subscription(LaserScan, scan_topic,  self.scan_callback, 10)

        self.create_timer(1.0 / rate_hz, self.control_loop)

        self.get_logger().info(
            f'Crab path follower ready — speed {self.speed:.3f} m/s, '
            f'tolerance {self.tolerance:.3f} m, stop_distance {self.stop_dist:.3f} m'
        )
        self.get_logger().info(
            f'Path: {path_topic} | Odom: {odom_topic} | '
            f'Scan: {scan_topic} | Cmd: {cmd_vel_topic}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        self.robot_x   = msg.pose.pose.position.x
        self.robot_y   = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        self.robot_yaw = _quaternion_to_yaw(o.x, o.y, o.z, o.w)
        self.odom_time = self.get_clock().now()

    def path_callback(self, msg: Path):
        self.latest_path      = msg
        self.latest_path_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        self.latest_scan      = msg
        self.latest_scan_time = self.get_clock().now()

    # ── Control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        now = self.get_clock().now()
        cmd = Twist()  # all-zero default → safe stop

        # ── Gate: path freshness ─────────────────────────────────────────────
        if self.latest_path is None or self.latest_path_time is None:
            self.cmd_pub.publish(cmd)
            return

        path_age = (now - self.latest_path_time).nanoseconds * 1e-9
        if path_age > self.path_timeout:
            self.cmd_pub.publish(cmd)
            return

        # ── Gate: stop path ──────────────────────────────────────────────────
        if len(self.latest_path.poses) < 2:
            self.cmd_pub.publish(cmd)
            return

        # ── Goal vector in world (odom) frame ───────────────────────────────
        goal = self.latest_path.poses[1].pose.position
        world_dx = goal.x - self.robot_x
        world_dy = goal.y - self.robot_y
        dist = math.sqrt(world_dx * world_dx + world_dy * world_dy)

        if dist < self.tolerance:
            self.cmd_pub.publish(cmd)
            return

        # ── Rotate goal vector: world frame → body frame ─────────────────────
        # Body frame: +x = forward, +y = left.  Robot has yaw = robot_yaw in world.
        # Rotation matrix (world → body):
        #   body_x =  world_dx * cos(yaw) + world_dy * sin(yaw)
        #   body_y = -world_dx * sin(yaw) + world_dy * cos(yaw)
        cos_y = math.cos(self.robot_yaw)
        sin_y = math.sin(self.robot_yaw)
        body_dx =  world_dx * cos_y + world_dy * sin_y
        body_dy = -world_dx * sin_y + world_dy * cos_y

        # Unit vector in body frame
        body_ux = body_dx / dist
        body_uy = body_dy / dist

        # ── Obstacle veto (in body / scan frame) ─────────────────────────────
        # The LiDAR is mounted on the body so scan angles are body-frame angles.
        # We check clearance along the commanded body-frame direction.
        cmd_angle = math.atan2(body_uy, body_ux)
        clearance = self._scan_clearance(cmd_angle)

        if clearance is not None and clearance < self.stop_dist:
            self.get_logger().warn(
                f'Obstacle at {clearance:.2f} m in commanded direction '
                f'{math.degrees(cmd_angle):.0f}° — holding position.',
                throttle_duration_sec=0.5,
            )
            self.cmd_pub.publish(cmd)
            return

        # ── Publish crab command ──────────────────────────────────────────────
        cmd.linear.x  = self.speed * body_ux
        cmd.linear.y  = self.speed * body_uy
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    # ── Scan clearance helper ─────────────────────────────────────────────────

    def _scan_clearance(self, body_angle_rad: float):
        """
        Minimum range within a window around body_angle_rad.
        Returns None if the scan is stale or unavailable (do not veto on None).
        """
        if self.latest_scan is None or self.latest_scan_time is None:
            return None

        scan_age = (self.get_clock().now() - self.latest_scan_time).nanoseconds * 1e-9
        if scan_age > self.scan_timeout:
            return None

        scan = self.latest_scan
        angle_inc = abs(scan.angle_increment)
        if angle_inc < 1e-9:
            return None

        window_beams = max(1, int(self._obstacle_window_rad / angle_inc))
        raw_idx = (body_angle_rad - scan.angle_min) / scan.angle_increment
        center = int(round(raw_idx))
        center = max(window_beams, min(len(scan.ranges) - 1 - window_beams, center))

        min_r = None
        for idx in range(center - window_beams, center + window_beams + 1):
            if idx < 0 or idx >= len(scan.ranges):
                continue
            r = scan.ranges[idx]
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min:
                r = scan.range_min
            elif r > scan.range_max:
                continue
            if min_r is None or r < min_r:
                min_r = r

        return min_r


def main(args=None):
    rclpy.init(args=args)
    node = CrabPathFollower()
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
