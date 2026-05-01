"""Generic LED controller utilities."""

import time

from .rpi_ledpixel import PwmLedStrip
from .spi_ledpixel import SpiLedStrip


class LedController:
    """Select and manage the onboard LED driver."""

    def __init__(
        self,
        driver='spi',
        count=7,
        brightness=64,
        sequence='GRB',
        spi_bus=0,
        spi_device=0,
        pwm_pin=18,
        pwm_freq_hz=800000,
        pwm_dma=10,
        pwm_invert=False,
        pwm_channel=0,
    ):
        self.driver_name = str(driver).strip().lower() or 'spi'
        self.count = max(1, int(count))
        self.brightness = max(0, min(255, int(brightness)))
        self.sequence = str(sequence).strip().upper() or 'GRB'
        self.spi_bus = int(spi_bus)
        self.spi_device = int(spi_device)
        self.pwm_pin = int(pwm_pin)
        self.pwm_freq_hz = int(pwm_freq_hz)
        self.pwm_dma = int(pwm_dma)
        self.pwm_invert = bool(pwm_invert)
        self.pwm_channel = int(pwm_channel)
        self.init_error = ''
        self.strip = None

        self._create_strip()

    @property
    def available(self):
        return self.strip is not None and self.init_error == ''

    def _create_strip(self):
        candidates = self._resolve_candidates(self.driver_name)
        for candidate in candidates:
            strip = self._build_strip(candidate)
            if strip is None:
                continue
            if self._strip_ready(strip, candidate):
                self.strip = strip
                self.driver_name = candidate
                self.init_error = ''
                return
            self.init_error = getattr(strip, 'init_error', '') or f'{candidate} backend unavailable.'

        self.strip = None
        if not self.init_error:
            self.init_error = 'No LED backend could be initialized.'

    def _resolve_candidates(self, driver_name):
        if driver_name == 'auto':
            return ['spi', 'pwm']
        if driver_name in {'spi', 'pwm'}:
            return [driver_name]
        return ['spi']

    def _build_strip(self, driver_name):
        if driver_name == 'spi':
            return SpiLedStrip(
                count=self.count,
                brightness=self.brightness,
                sequence=self.sequence,
                bus=self.spi_bus,
                device=self.spi_device,
            )
        if driver_name == 'pwm':
            return PwmLedStrip(
                count=self.count,
                brightness=self.brightness,
                sequence=self.sequence,
                pin=self.pwm_pin,
                freq_hz=self.pwm_freq_hz,
                dma=self.pwm_dma,
                invert=self.pwm_invert,
                channel=self.pwm_channel,
            )
        return None

    def _strip_ready(self, strip, driver_name):
        if driver_name == 'spi':
            return bool(strip.check_spi_state())
        if driver_name == 'pwm':
            return bool(strip.check_rpi_ws281x_state())
        return False

    def show_color(self, color):
        if not self.available:
            return
        self.strip.set_all_led_rgb(color)

    def clear(self):
        self.show_color([0, 0, 0])

    def color_wipe(self, color, wait_ms=50):
        if not self.available:
            return
        for index in range(self.strip.get_led_count()):
            self.strip.set_led_rgb_data(index, color)
            self.strip.show()
            time.sleep(wait_ms / 1000.0)

    def set_index_mask(self, index_mask, red, green, blue):
        if not self.available:
            return
        color = [int(red), int(green), int(blue)]
        for index in range(self.strip.get_led_count()):
            if (index_mask >> index) & 0x01:
                self.strip.set_led_rgb_data(index, color)
        self.strip.show()

    def close(self):
        if self.strip is not None:
            self.strip.led_close()
            self.strip = None


# Backward-compatible alias for older imports.
Led = LedController
