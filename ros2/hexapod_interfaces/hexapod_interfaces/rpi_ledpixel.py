"""PWM-backed WS2812 LED strip helper."""

try:
    from rpi_ws281x import Adafruit_NeoPixel, Color
except ImportError:  # pragma: no cover - depends on robot hardware packages.
    Adafruit_NeoPixel = None
    Color = None


class PwmLedStrip:
    """Drive a WS2812-compatible LED strip with the rpi_ws281x backend."""

    _LED_TYPE_OFFSETS = {
        'RGB': (0, 1, 2),
        'RBG': (0, 2, 1),
        'GRB': (1, 0, 2),
        'GBR': (2, 0, 1),
        'BRG': (1, 2, 0),
        'BGR': (2, 1, 0),
    }

    def __init__(
        self,
        count=8,
        brightness=255,
        sequence='RGB',
        pin=18,
        freq_hz=800000,
        dma=10,
        invert=False,
        channel=0,
    ):
        self.pin = int(pin)
        self.freq_hz = int(freq_hz)
        self.dma = int(dma)
        self.invert = bool(invert)
        self.channel = int(channel)
        self.strip = None
        self.init_error = ''
        self.led_init_state = 0

        self.set_led_type(sequence)
        self.set_led_count(count)
        self.set_led_brightness(brightness)
        self.led_begin()
        if self.led_init_state:
            self.set_all_led_color(0, 0, 0)

    def led_begin(self):
        if Adafruit_NeoPixel is None:
            self.init_error = 'rpi_ws281x is not installed.'
            self.led_init_state = 0
            return

        try:
            self.strip = Adafruit_NeoPixel(
                self.get_led_count(),
                self.pin,
                self.freq_hz,
                self.dma,
                self.invert,
                self.led_brightness,
                self.channel,
            )
            self.strip.begin()
            self.led_init_state = 1
            self.init_error = ''
        except Exception as exc:  # pragma: no cover - depends on robot hardware.
            self.init_error = f'Failed to initialize rpi_ws281x output: {exc}'
            self.led_init_state = 0

    def check_rpi_ws281x_state(self):
        return self.led_init_state

    def led_close(self):
        if self.led_init_state:
            self.set_all_led_rgb([0, 0, 0])
        self.strip = None
        self.led_init_state = 0

    def set_led_count(self, count):
        self.led_count = max(1, int(count))
        self.led_color = [0] * (self.led_count * 3)
        self.led_original_color = [0] * (self.led_count * 3)

    def get_led_count(self):
        return self.led_count

    def set_led_type(self, rgb_type):
        offsets = self._LED_TYPE_OFFSETS.get(str(rgb_type).upper())
        if offsets is None:
            offsets = self._LED_TYPE_OFFSETS['RGB']
        self.led_red_offset, self.led_green_offset, self.led_blue_offset = offsets

    def set_led_brightness(self, brightness):
        self.led_brightness = max(0, min(255, int(brightness)))
        for index in range(self.get_led_count()):
            start = index * 3
            self.set_led_rgb_data(index, self.led_original_color[start:start + 3])

    def set_ledpixel(self, index, red, green, blue):
        if index < 0 or index >= self.led_count:
            return

        red = max(0, min(255, int(red)))
        green = max(0, min(255, int(green)))
        blue = max(0, min(255, int(blue)))

        scaled = [0, 0, 0]
        scaled[self.led_red_offset] = round(red * self.led_brightness / 255)
        scaled[self.led_green_offset] = round(green * self.led_brightness / 255)
        scaled[self.led_blue_offset] = round(blue * self.led_brightness / 255)

        base = index * 3
        self.led_original_color[base + self.led_red_offset] = red
        self.led_original_color[base + self.led_green_offset] = green
        self.led_original_color[base + self.led_blue_offset] = blue

        for channel in range(3):
            self.led_color[base + channel] = scaled[channel]

    def set_led_color_data(self, index, red, green, blue):
        self.set_ledpixel(index, red, green, blue)

    def set_led_rgb_data(self, index, color):
        self.set_ledpixel(index, color[0], color[1], color[2])

    def set_led_color(self, index, red, green, blue):
        self.set_ledpixel(index, red, green, blue)
        self.show()

    def set_led_rgb(self, index, color):
        self.set_led_rgb_data(index, color)
        self.show()

    def set_all_led_color_data(self, red, green, blue):
        for index in range(self.get_led_count()):
            self.set_led_color_data(index, red, green, blue)

    def set_all_led_rgb_data(self, color):
        for index in range(self.get_led_count()):
            self.set_led_rgb_data(index, color)

    def set_all_led_color(self, red, green, blue):
        self.set_all_led_color_data(red, green, blue)
        self.show()

    def set_all_led_rgb(self, color):
        self.set_all_led_rgb_data(color)
        self.show()

    def show(self):
        if not self.led_init_state or self.strip is None or Color is None:
            return

        for index in range(self.get_led_count()):
            base = index * 3
            self.strip.setPixelColor(
                index,
                Color(
                    self.led_color[base],
                    self.led_color[base + 1],
                    self.led_color[base + 2],
                ),
            )
        self.strip.show()

    def wheel(self, position):
        position = int(position) % 256
        if position < 85:
            return [(255 - position * 3), (position * 3), 0]
        if position < 170:
            position -= 85
            return [0, (255 - position * 3), (position * 3)]
        position -= 170
        return [(position * 3), 0, (255 - position * 3)]

    def hsv2rgb(self, hue, saturation, value):
        hue = int(hue) % 360
        saturation = max(0, min(100, int(saturation)))
        value = max(0, min(100, int(value)))

        rgb_max = round(value * 2.55)
        rgb_min = round(rgb_max * (100 - saturation) / 100)
        sector = round(hue / 60)
        diff = round(hue % 60)
        rgb_adj = round((rgb_max - rgb_min) * diff / 60)

        if sector == 0:
            return [rgb_max, rgb_min + rgb_adj, rgb_min]
        if sector == 1:
            return [rgb_max - rgb_adj, rgb_max, rgb_min]
        if sector == 2:
            return [rgb_min, rgb_max, rgb_min + rgb_adj]
        if sector == 3:
            return [rgb_min, rgb_max - rgb_adj, rgb_max]
        if sector == 4:
            return [rgb_min + rgb_adj, rgb_min, rgb_max]
        return [rgb_max, rgb_min, rgb_max - rgb_adj]
