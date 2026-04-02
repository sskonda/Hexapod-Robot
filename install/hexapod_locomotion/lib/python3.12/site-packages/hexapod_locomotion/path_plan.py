#!/usr/bin/env python3

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass(frozen=True)
class MotionSegment:
    name: str
    linear_x_mps: float
    duration_sec: float


class PathPlanNode(Node):
    def __init__(self):
        super().__init__('path_plan')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('startup_delay_sec', 1.0)
        self.declare_parameter('forward_distance_m', 0.5)
        self.declare_parameter('backward_distance_m', 0.5)
        self.declare_parameter('linear_speed_mps', 0.05)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.startup_delay_sec = max(0.0, float(self.get_parameter('startup_delay_sec').value))
        self.forward_distance_m = max(0.0, float(self.get_parameter('forward_distance_m').value))
        self.backward_distance_m = max(0.0, float(self.get_parameter('backward_distance_m').value))
        self.linear_speed_mps = max(0.001, abs(float(self.get_parameter('linear_speed_mps').value)))

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.segments = self.build_segments()
        self.current_segment_index = -1
        self.segment_started_at = self.get_clock().now()
        self.started = False
        self.finished = False

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.control_loop)

        self.get_logger().info(
            'Path planner ready: startup delay '
            f'{self.startup_delay_sec:.2f}s, forward {self.forward_distance_m:.2f}m, '
            f'backward {self.backward_distance_m:.2f}m at {self.linear_speed_mps:.2f}m/s'
        )

        if not self.segments:
            self.get_logger().warn('No motion segments configured. The node will publish stop and exit.')

    def build_segments(self):
        segments = []
        if self.forward_distance_m > 0.0:
            segments.append(
                MotionSegment(
                    name='forward',
                    linear_x_mps=self.linear_speed_mps,
                    duration_sec=self.forward_distance_m / self.linear_speed_mps,
                )
            )
        if self.backward_distance_m > 0.0:
            segments.append(
                MotionSegment(
                    name='backward',
                    linear_x_mps=-self.linear_speed_mps,
                    duration_sec=self.backward_distance_m / self.linear_speed_mps,
                )
            )
        return segments

    def elapsed_since(self, start_time):
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

    def current_segment(self):
        if 0 <= self.current_segment_index < len(self.segments):
            return self.segments[self.current_segment_index]
        return None

    def publish_linear_velocity(self, linear_x_mps):
        msg = Twist()
        msg.linear.x = float(linear_x_mps)
        self.publisher.publish(msg)

    def publish_stop(self):
        self.publish_linear_velocity(0.0)

    def start_next_segment(self):
        self.current_segment_index += 1
        self.segment_started_at = self.get_clock().now()

        segment = self.current_segment()
        if segment is None:
            self.finish_plan()
            return

        self.get_logger().info(
            f'Starting segment "{segment.name}" for {segment.duration_sec:.2f}s '
            f'at {segment.linear_x_mps:.2f}m/s'
        )

    def finish_plan(self):
        if self.finished:
            return

        self.finished = True
        self.publish_stop()
        self.get_logger().info('Path plan complete. Published stop command.')
        self.timer.cancel()
        rclpy.shutdown()

    def control_loop(self):
        if self.finished:
            return

        if not self.started:
            self.publish_stop()
            if self.elapsed_since(self.segment_started_at) < self.startup_delay_sec:
                return

            self.started = True
            self.start_next_segment()
            return

        segment = self.current_segment()
        if segment is None:
            self.finish_plan()
            return

        if self.elapsed_since(self.segment_started_at) >= segment.duration_sec:
            self.publish_stop()
            self.start_next_segment()
            return

        self.publish_linear_velocity(segment.linear_x_mps)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()
