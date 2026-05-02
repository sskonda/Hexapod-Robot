"""Set the onboard LED matrix to a solid color with optional QR trigger."""

import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .led import LedController


NAMED_COLORS = {
    'black': [0, 0, 0],
    'white': [255, 255, 255],
    'red': [255, 0, 0],
    'green': [0, 255, 0],
    'blue': [0, 0, 255],
    'yellow': [255, 255, 0],
    'cyan': [0, 255, 255],
    'aqua': [0, 255, 255],
    'magenta': [255, 0, 255],
    'purple': [128, 0, 128],
    'orange': [255, 165, 0],
}


def clamp_byte(value):
    return max(0, min(255, int(round(float(value)))))


def brightness_percent_to_byte(percent):
    return clamp_byte(max(0.0, min(100.0, float(percent))) * 255.0 / 100.0)


def parse_color(color_value):
    """Parse a named, hex, or comma-separated RGB color into [r, g, b]."""

    color_text = str(color_value).strip()
    color_key = color_text.lower().replace(' ', '_').replace('-', '_')
    if color_key in NAMED_COLORS:
        return list(NAMED_COLORS[color_key])

    hex_text = color_text[1:] if color_text.startswith('#') else color_text
    if re.fullmatch(r'[0-9a-fA-F]{3}', hex_text):
        return [
            int(hex_text[0] * 2, 16),
            int(hex_text[1] * 2, 16),
            int(hex_text[2] * 2, 16),
        ]
    if re.fullmatch(r'[0-9a-fA-F]{6}', hex_text):
        return [
            int(hex_text[0:2], 16),
            int(hex_text[2:4], 16),
            int(hex_text[4:6], 16),
        ]

    rgb_match = re.fullmatch(
        r'(?:rgb\()?'
        r'\s*([0-9]+(?:\.[0-9]+)?)\s*[, ]\s*'
        r'([0-9]+(?:\.[0-9]+)?)\s*[, ]\s*'
        r'([0-9]+(?:\.[0-9]+)?)\s*\)?',
        color_text,
        flags=re.IGNORECASE,
    )
    if rgb_match:
        return [clamp_byte(channel) for channel in rgb_match.groups()]

    raise ValueError(
        f'Unsupported LED color "{color_text}". Use a name, #RRGGBB, or R,G,B.'
    )


class LedMatrixNode(Node):
    """Set every LED in the robot matrix/strip to one selected color."""

    def __init__(self):
        super().__init__('led_matrix')

        self.declare_parameter('color', 'cyan')
        self.declare_parameter('qr_trigger_text_topic', '')
        self.declare_parameter('qr_trigger_text', 'blue')
        self.declare_parameter('qr_trigger_color', 'cyan')
        self.declare_parameter('qr_trigger_timeout_sec', 10.0)
        self.declare_parameter('arm_command_topic', '')
        self.declare_parameter('arm_command_on_value', 'True')
        self.declare_parameter('arm_command_off_value', 'False')
        self.declare_parameter('brightness_percent', 75.0)
        self.declare_parameter('led_backend', 'spi')
        self.declare_parameter('led_count', 7)
        self.declare_parameter('led_sequence', 'GRB')
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('pwm_pin', 18)
        self.declare_parameter('pwm_freq_hz', 800000)
        self.declare_parameter('pwm_dma', 10)
        self.declare_parameter('pwm_invert', False)
        self.declare_parameter('pwm_channel', 0)
        self.declare_parameter('reapply_period_sec', 0.0)
        self.declare_parameter('dry_run', False)

        self.color = self._read_color_parameter('color', 'cyan')
        self.inactive_color = list(self.color)
        self.qr_trigger_color = self._read_color_parameter(
            'qr_trigger_color',
            'cyan',
        )
        self.qr_trigger_text_topic = str(
            self.get_parameter('qr_trigger_text_topic').value
        ).strip()
        self.qr_trigger_text = self.normalize_trigger_text(
            self.get_parameter('qr_trigger_text').value
        )
        self.qr_trigger_timeout_sec = max(
            0.0,
            float(self.get_parameter('qr_trigger_timeout_sec').value),
        )
        self.arm_command_topic = str(
            self.get_parameter('arm_command_topic').value
        ).strip()
        self.arm_command_on_value = str(
            self.get_parameter('arm_command_on_value').value
        )
        self.arm_command_off_value = str(
            self.get_parameter('arm_command_off_value').value
        )
        self.brightness_percent = max(
            0.0,
            min(100.0, float(self.get_parameter('brightness_percent').value)),
        )
        self.brightness = brightness_percent_to_byte(self.brightness_percent)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.led = None
        self.arm_command_pub = None
        self.qr_trigger_sub = None
        self.qr_trigger_timer = None
        self.qr_trigger_active = False
        self.last_qr_trigger_seen_sec = None
        self._reported_status = False

        if not self.dry_run:
            self.led = LedController(
                driver=str(self.get_parameter('led_backend').value),
                count=int(self.get_parameter('led_count').value),
                brightness=self.brightness,
                sequence=str(self.get_parameter('led_sequence').value),
                spi_bus=int(self.get_parameter('spi_bus').value),
                spi_device=int(self.get_parameter('spi_device').value),
                pwm_pin=int(self.get_parameter('pwm_pin').value),
                pwm_freq_hz=int(self.get_parameter('pwm_freq_hz').value),
                pwm_dma=int(self.get_parameter('pwm_dma').value),
                pwm_invert=bool(self.get_parameter('pwm_invert').value),
                pwm_channel=int(self.get_parameter('pwm_channel').value),
            )

        self._apply_color()
        if self.arm_command_topic:
            self.arm_command_pub = self.create_publisher(
                String,
                self.arm_command_topic,
                10,
            )
            self.publish_arm_command(self.arm_command_off_value)
        if self.qr_trigger_text_topic and self.qr_trigger_text:
            self.qr_trigger_sub = self.create_subscription(
                String,
                self.qr_trigger_text_topic,
                self.qr_trigger_callback,
                10,
            )
            self.qr_trigger_timer = self.create_timer(
                0.25,
                self.check_qr_trigger_timeout,
            )
            self.get_logger().info(
                f'Watching {self.qr_trigger_text_topic} for QR text '
                f'"{self.qr_trigger_text}" to set RGB {self.qr_trigger_color}.'
            )

        reapply_period_sec = max(
            0.0,
            float(self.get_parameter('reapply_period_sec').value),
        )
        self.reapply_timer = None
        if reapply_period_sec > 0.0:
            self.reapply_timer = self.create_timer(
                reapply_period_sec,
                self._apply_color,
            )

    def destroy_node(self):
        if self.qr_trigger_active:
            self.deactivate_qr_trigger('shutdown')
        if self.led is not None:
            self.led.close()
        return super().destroy_node()

    def _read_color_parameter(self, parameter_name, fallback_color_name):
        color_value = self.get_parameter(parameter_name).value
        try:
            return parse_color(color_value)
        except ValueError as exc:
            self.get_logger().warn(
                f'{exc} Falling back to {fallback_color_name}.'
            )
            return list(NAMED_COLORS[fallback_color_name])

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def normalize_trigger_text(self, value):
        return str(value).strip().lower()

    def qr_trigger_callback(self, msg: String):
        if self.normalize_trigger_text(msg.data) != self.qr_trigger_text:
            return

        self.last_qr_trigger_seen_sec = self.now_sec()
        if self.qr_trigger_active:
            return

        self.qr_trigger_active = True
        self.publish_arm_command(self.arm_command_on_value)
        self.set_color(
            self.qr_trigger_color,
            f'QR text "{self.qr_trigger_text}"',
        )

    def check_qr_trigger_timeout(self):
        if (
            not self.qr_trigger_active
            or self.last_qr_trigger_seen_sec is None
            or self.qr_trigger_timeout_sec <= 0.0
        ):
            return

        if self.now_sec() - self.last_qr_trigger_seen_sec >= self.qr_trigger_timeout_sec:
            self.deactivate_qr_trigger('QR timeout')

    def deactivate_qr_trigger(self, reason):
        self.qr_trigger_active = False
        self.last_qr_trigger_seen_sec = None
        self.publish_arm_command(self.arm_command_off_value)
        self.set_color(self.inactive_color, reason)

    def publish_arm_command(self, value):
        if self.arm_command_pub is None:
            return

        self.arm_command_pub.publish(String(data=str(value)))

    def set_color(self, color, source):
        self.color = list(color)
        self._reported_status = False
        self._apply_color(source=source)

    def _apply_color(self, source='parameter'):
        if self.dry_run:
            if not self._reported_status:
                self.get_logger().info(
                    f'Dry run: would set LED matrix to RGB {self.color} '
                    f'from {source} at {self.brightness_percent:.1f}% brightness.'
                )
                self._reported_status = True
            return

        if self.led is None or not self.led.available:
            init_error = 'unknown LED initialization error'
            if self.led is not None and self.led.init_error:
                init_error = self.led.init_error
            if not self._reported_status:
                self.get_logger().warn(
                    f'LED backend unavailable; matrix color was not applied: {init_error}'
                )
                self._reported_status = True
            return

        self.led.show_color(self.color)
        if not self._reported_status:
            self.get_logger().info(
                f'Set LED matrix to RGB {self.color} from {source} at '
                f'{self.brightness_percent:.1f}% brightness '
                f'using {self.led.driver_name} mode with {self.led.count} pixels.'
            )
            self._reported_status = True


def main(args=None):
    rclpy.init(args=args)
    node = LedMatrixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
