"""Map camera scene colors to the onboard LED strip."""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from .led import LedController


def compute_scene_color_bgr(
    frame,
    sample_width=32,
    sample_height=24,
    min_saturation=40,
    min_value=32,
):
    """Return a representative scene color in BGR order."""

    if frame is None or frame.size == 0:
        return None

    reduced = cv2.resize(
        frame,
        (max(1, int(sample_width)), max(1, int(sample_height))),
        interpolation=cv2.INTER_AREA,
    )
    hsv = cv2.cvtColor(reduced, cv2.COLOR_BGR2HSV)

    saturation = hsv[:, :, 1].astype(np.float32)
    value = hsv[:, :, 2].astype(np.float32)
    weights = np.maximum(1.0, saturation) * np.maximum(1.0, value)
    mask = (saturation >= float(min_saturation)) & (value >= float(min_value))

    pixels = reduced[mask] if np.any(mask) else reduced.reshape(-1, 3)
    if pixels.size == 0:
        return None

    masked_weights = weights[mask] if np.any(mask) else weights.reshape(-1)
    scene_color = np.average(
        pixels.astype(np.float32),
        axis=0,
        weights=masked_weights.astype(np.float32),
    )
    return np.clip(scene_color, 0, 255)


class CameraLedNode(Node):
    """Subscribe to a camera topic and mirror the scene color on the LEDs."""

    def __init__(self):
        super().__init__('camera_led')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('led_backend', 'spi')
        self.declare_parameter('led_count', 7)
        self.declare_parameter('led_brightness', 64)
        self.declare_parameter('led_sequence', 'GRB')
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('pwm_pin', 18)
        self.declare_parameter('pwm_freq_hz', 800000)
        self.declare_parameter('pwm_dma', 10)
        self.declare_parameter('pwm_invert', False)
        self.declare_parameter('pwm_channel', 0)
        self.declare_parameter('sample_width', 32)
        self.declare_parameter('sample_height', 24)
        self.declare_parameter('min_saturation', 40)
        self.declare_parameter('min_value', 32)
        self.declare_parameter('smoothing_factor', 0.35)
        self.declare_parameter('update_period_sec', 0.05)
        self.declare_parameter('min_color_delta', 3)
        self.declare_parameter('clear_on_shutdown', True)
        self.declare_parameter('dry_run', False)

        image_topic = str(self.get_parameter('image_topic').value)
        qos_depth = int(self.get_parameter('qos_depth').value)

        self.sample_width = max(1, int(self.get_parameter('sample_width').value))
        self.sample_height = max(1, int(self.get_parameter('sample_height').value))
        self.min_saturation = max(0, int(self.get_parameter('min_saturation').value))
        self.min_value = max(0, int(self.get_parameter('min_value').value))
        self.smoothing_factor = min(
            1.0,
            max(0.0, float(self.get_parameter('smoothing_factor').value)),
        )
        self.update_period_sec = max(
            0.0,
            float(self.get_parameter('update_period_sec').value),
        )
        self.min_color_delta = max(0, int(self.get_parameter('min_color_delta').value))
        self.clear_on_shutdown = bool(self.get_parameter('clear_on_shutdown').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self.bridge = CvBridge()
        self.last_update_monotonic = 0.0
        self.smoothed_color_bgr = None
        self.last_applied_rgb = None

        self.led = None
        if not self.dry_run:
            self.led = LedController(
                driver=str(self.get_parameter('led_backend').value),
                count=int(self.get_parameter('led_count').value),
                brightness=int(self.get_parameter('led_brightness').value),
                sequence=str(self.get_parameter('led_sequence').value),
                spi_bus=int(self.get_parameter('spi_bus').value),
                spi_device=int(self.get_parameter('spi_device').value),
                pwm_pin=int(self.get_parameter('pwm_pin').value),
                pwm_freq_hz=int(self.get_parameter('pwm_freq_hz').value),
                pwm_dma=int(self.get_parameter('pwm_dma').value),
                pwm_invert=bool(self.get_parameter('pwm_invert').value),
                pwm_channel=int(self.get_parameter('pwm_channel').value),
            )

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_depth,
        )

        self.get_logger().info(f'Subscribed to camera topic {image_topic}')
        if self.dry_run:
            self.get_logger().warn('Camera LED node is running in dry-run mode.')
        elif self.led is None or not self.led.available:
            init_error = 'unknown LED initialization error'
            if self.led is not None and self.led.init_error:
                init_error = self.led.init_error
            self.get_logger().warn(
                f'LED backend is unavailable, so colors will not be shown on hardware: {init_error}'
            )
        else:
            self.get_logger().info(
                f'LED backend ready using {self.led.driver_name} mode with {self.led.count} pixels.'
            )

    def destroy_node(self):
        if self.led is not None:
            if self.clear_on_shutdown and self.led.available:
                self.led.clear()
            self.led.close()
        return super().destroy_node()

    def image_callback(self, msg):
        now = time.monotonic()
        if (
            self.update_period_sec > 0.0
            and now - self.last_update_monotonic < self.update_period_sec
        ):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        scene_color_bgr = compute_scene_color_bgr(
            frame,
            sample_width=self.sample_width,
            sample_height=self.sample_height,
            min_saturation=self.min_saturation,
            min_value=self.min_value,
        )
        if scene_color_bgr is None:
            return

        if self.smoothed_color_bgr is None:
            self.smoothed_color_bgr = scene_color_bgr.astype(np.float32)
        else:
            alpha = self.smoothing_factor
            self.smoothed_color_bgr = (
                (1.0 - alpha) * self.smoothed_color_bgr
                + alpha * scene_color_bgr.astype(np.float32)
            )

        rgb = [
            int(round(self.smoothed_color_bgr[2])),
            int(round(self.smoothed_color_bgr[1])),
            int(round(self.smoothed_color_bgr[0])),
        ]

        if (
            self.last_applied_rgb is not None
            and max(abs(rgb[index] - self.last_applied_rgb[index]) for index in range(3))
            < self.min_color_delta
        ):
            return

        self.last_applied_rgb = rgb
        self.last_update_monotonic = now

        if self.led is not None and self.led.available:
            self.led.show_color(rgb)


def main(args=None):
    rclpy.init(args=args)
    node = CameraLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
