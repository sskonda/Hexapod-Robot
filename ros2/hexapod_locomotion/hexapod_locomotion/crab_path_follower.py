#!/usr/bin/env python3
"""Crab-motion path follower for the hexapod robot.

Subscribes to a nav_msgs/Path published by the gap-following explorer and
converts the 2-pose rolling path into a geometry_msgs/Twist command.

Key design constraint: the robot body NEVER rotates.  The odom frame and
the body frame are always aligned, so the world-frame goal vector can be
commanded directly as linear.x / linear.y without any frame transform.
angular.z is always zero.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


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

        path_topic         = str(self.get_parameter('path_topic').value)
        odom_topic         = str(self.get_parameter('odom_topic').value)
        cmd_vel_topic      = str(self.get_parameter('cmd_vel_topic').value)
        self.speed         = max(0.001, float(self.get_parameter('constant_speed_mps').value))
        self.tolerance     = max(0.01,  float(self.get_parameter('goal_tolerance_m').value))
        self.path_timeout  = max(0.1,   float(self.get_parameter('path_timeout_sec').value))
        rate_hz            = max(1.0,   float(self.get_parameter('cmd_vel_rate_hz').value))

        # ── State ────────────────────────────────────────────────────────────
        self.latest_path      = None
        self.latest_path_time = None
        self.robot_x          = 0.0
        self.robot_y          = 0.0

        # ── Publishers / Subscribers ─────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.create_subscription(Path,     path_topic, self.path_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.create_timer(1.0 / rate_hz, self.control_loop)

        self.get_logger().info(
            f'Crab path follower ready — speed {self.speed:.3f} m/s, '
            f'tolerance {self.tolerance:.3f} m'
        )
        self.get_logger().info(
            f'Subscribed to {path_topic} and {odom_topic}, publishing on {cmd_vel_topic}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def path_callback(self, msg: Path):
        self.latest_path      = msg
        self.latest_path_time = self.get_clock().now()

    # ── Control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        cmd = Twist()  # all-zero default → safe stop

        # No path received yet
        if self.latest_path is None or self.latest_path_time is None:
            self.cmd_pub.publish(cmd)
            return

        # Path is stale — stop
        age_sec = (self.get_clock().now() - self.latest_path_time).nanoseconds * 1e-9
        if age_sec > self.path_timeout:
            self.cmd_pub.publish(cmd)
            return

        # A stop-path has only one pose (explorer publishes this when replanning)
        if len(self.latest_path.poses) < 2:
            self.cmd_pub.publish(cmd)
            return

        # Goal is the second pose in the 2-pose rolling path
        goal = self.latest_path.poses[1].pose.position
        dx   = goal.x - self.robot_x
        dy   = goal.y - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Within tolerance → hold position and wait for a new path
        if dist < self.tolerance:
            self.cmd_pub.publish(cmd)
            return

        # ── Crab command ──────────────────────────────────────────────────────
        # The robot body never rotates, so odom frame == body frame at all times.
        # The goal vector in odom space is therefore also the body-frame velocity
        # direction.  Normalise to unit length then scale by the constant speed.
        cmd.linear.x  = self.speed * (dx / dist)
        cmd.linear.y  = self.speed * (dy / dist)
        cmd.angular.z = 0.0   # never rotate

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
