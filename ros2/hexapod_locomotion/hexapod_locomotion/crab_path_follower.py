#!/usr/bin/env python3
"""Crab-motion path follower for the hexapod robot.

Subscribes to a nav_msgs/Path published by the gap-following explorer and
converts the 2-pose rolling path into a geometry_msgs/Twist command.

The explorer publishes goals in the odom frame, but locomotion interprets
``cmd_vel`` in the robot body frame.  On a real hexapod the body can yaw a
few degrees due to slip or uneven gait cycles, so the follower must rotate
the odom-frame goal vector back into the current body frame before sending
linear commands.  The robot is still intended to crab rather than rotate to
face the travel direction, so the yaw controller acts as a heading hold that
counteracts drift instead of commanding the body to face the path heading.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def world_vector_to_body_frame(dx_world: float, dy_world: float, yaw_rad: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        cos_yaw * dx_world + sin_yaw * dy_world,
        -sin_yaw * dx_world + cos_yaw * dy_world,
    )


class CrabPathFollower(Node):
    def __init__(self):
        super().__init__('crab_path_follower')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('path_topic',          'path')
        self.declare_parameter('odom_topic',          'odom')
        self.declare_parameter('cmd_vel_topic',       'cmd_vel')
        self.declare_parameter('constant_speed_mps',  0.04)
        self.declare_parameter('goal_tolerance_m',    0.08)
        self.declare_parameter('path_timeout_sec',    1.0)
        self.declare_parameter('cmd_vel_rate_hz',     20.0)
        self.declare_parameter('yaw_correction_gain', 0.6)
        self.declare_parameter('max_angular_speed_rad_s', 0.12)
        self.declare_parameter('yaw_deadband_deg', 5.0)

        path_topic         = str(self.get_parameter('path_topic').value)
        odom_topic         = str(self.get_parameter('odom_topic').value)
        cmd_vel_topic      = str(self.get_parameter('cmd_vel_topic').value)
        self.speed         = max(0.001, float(self.get_parameter('constant_speed_mps').value))
        self.tolerance     = max(0.01,  float(self.get_parameter('goal_tolerance_m').value))
        self.path_timeout  = max(0.1,   float(self.get_parameter('path_timeout_sec').value))
        rate_hz            = max(1.0,   float(self.get_parameter('cmd_vel_rate_hz').value))
        self.yaw_gain      = max(0.0,   float(self.get_parameter('yaw_correction_gain').value))
        self.max_yaw_rate  = max(0.0,   float(self.get_parameter('max_angular_speed_rad_s').value))
        self.yaw_deadband  = math.radians(
            max(0.0, float(self.get_parameter('yaw_deadband_deg').value))
        )

        # ── State ────────────────────────────────────────────────────────────
        self.latest_path      = None
        self.latest_path_time = None
        self.robot_x          = 0.0
        self.robot_y          = 0.0
        self.robot_yaw        = 0.0
        self.heading_hold_yaw = None

        # ── Publishers / Subscribers ─────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.create_subscription(Path,     path_topic, self.path_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.create_timer(1.0 / rate_hz, self.control_loop)

        self.get_logger().info(
            f'Crab path follower ready — speed {self.speed:.3f} m/s, '
            f'tolerance {self.tolerance:.3f} m, '
            f'max yaw correction {self.max_yaw_rate:.2f} rad/s'
        )
        self.get_logger().info(
            f'Subscribed to {path_topic} and {odom_topic}, publishing on {cmd_vel_topic}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.robot_yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

    def path_callback(self, msg: Path):
        self.latest_path      = msg
        self.latest_path_time = self.get_clock().now()

    # ── Control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        cmd = Twist()  # all-zero default → safe stop

        # No path received yet
        if self.latest_path is None or self.latest_path_time is None:
            self.heading_hold_yaw = None
            self.cmd_pub.publish(cmd)
            return

        # Path is stale — stop
        age_sec = (self.get_clock().now() - self.latest_path_time).nanoseconds * 1e-9
        if age_sec > self.path_timeout:
            self.heading_hold_yaw = None
            self.cmd_pub.publish(cmd)
            return

        # A stop-path has only one pose (explorer publishes this when replanning)
        if len(self.latest_path.poses) < 2:
            self.heading_hold_yaw = None
            self.cmd_pub.publish(cmd)
            return

        # Goal is the second pose in the 2-pose rolling path
        goal_pose = self.latest_path.poses[1].pose
        goal = goal_pose.position
        dx   = goal.x - self.robot_x
        dy   = goal.y - self.robot_y
        dist = math.hypot(dx, dy)

        # Within tolerance → hold position and wait for a new path
        if dist < self.tolerance:
            self.heading_hold_yaw = None
            self.cmd_pub.publish(cmd)
            return

        # Rotate the odom-frame goal vector into the robot body frame before
        # commanding locomotion.  This keeps the translation aligned with the
        # real robot even if the body yaws slightly relative to odom.
        if self.heading_hold_yaw is None:
            self.heading_hold_yaw = self.robot_yaw

        body_dx, body_dy = world_vector_to_body_frame(dx, dy, self.robot_yaw)
        body_dist = math.hypot(body_dx, body_dy)
        if body_dist < 1e-6:
            self.cmd_pub.publish(cmd)
            return

        cmd.linear.x = self.speed * (body_dx / body_dist)
        cmd.linear.y = self.speed * (body_dy / body_dist)

        yaw_error = normalize_angle(self.heading_hold_yaw - self.robot_yaw)
        if self.max_yaw_rate > 0.0 and abs(yaw_error) > self.yaw_deadband:
            cmd.angular.z = clamp(
                self.yaw_gain * yaw_error,
                -self.max_yaw_rate,
                self.max_yaw_rate,
            )

        self.cmd_pub.publish(cmd)


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
