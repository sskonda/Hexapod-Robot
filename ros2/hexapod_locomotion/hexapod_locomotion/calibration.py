#!/usr/bin/env python3

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from .calibration_store import JOINT_NAMES, load_offsets, save_offsets, zero_offsets


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration')

        share_dir = Path(get_package_share_directory('hexapod_locomotion'))
        default_yaml = str(share_dir / 'config' / 'servo_calibration.yaml')

        self.declare_parameter('calibration_file', default_yaml)
        self.calibration_file = self.get_parameter('calibration_file').value

        loaded = load_offsets(self.calibration_file)
        for joint in JOINT_NAMES:
            self.declare_parameter(joint, loaded[joint])

        self.publisher = self.create_publisher(JointState, '/servo_targets', 10)
        self.timer = self.create_timer(0.2, self.publish_current_pose)

        self.save_srv = self.create_service(Trigger, 'save_calibration', self.save_callback)
        self.load_srv = self.create_service(Trigger, 'load_calibration', self.load_callback)
        self.reset_srv = self.create_service(Trigger, 'reset_calibration', self.reset_callback)

        self.get_logger().info('Calibration node started')
        self.get_logger().info('Publishing current calibration pose to /servo_targets')
        self.get_logger().info('Use ros2 param set /calibration <joint_name> <value> to tweak offsets')
        self.get_logger().info('Use ros2 service call /save_calibration std_srvs/srv/Trigger {} to save')

    def current_offsets(self):
        return {joint: float(self.get_parameter(joint).value) for joint in JOINT_NAMES}

    def publish_current_pose(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [self.get_parameter(joint).value for joint in JOINT_NAMES]
        self.publisher.publish(msg)

    def save_callback(self, request, response):
        try:
            save_offsets(self.calibration_file, self.current_offsets())
            response.success = True
            response.message = f'Saved calibration to {self.calibration_file}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def load_callback(self, request, response):
        try:
            loaded = load_offsets(self.calibration_file)
            params = [Parameter(name, Parameter.Type.DOUBLE, float(value))
                      for name, value in loaded.items()]
            self.set_parameters(params)
            response.success = True
            response.message = f'Loaded calibration from {self.calibration_file}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def reset_callback(self, request, response):
        try:
            zeros = zero_offsets()
            params = [Parameter(name, Parameter.Type.DOUBLE, float(value))
                      for name, value in zeros.items()]
            self.set_parameters(params)
            response.success = True
            response.message = 'Reset all calibration offsets to zero'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
