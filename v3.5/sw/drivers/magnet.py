"""Tell a package's north pole from its south pole.

One ratiometric (analogue) Hall sensor on an ADC pin. With no magnet nearby
it idles near half supply; a north pole pushes the output one way and a
south pole the other. Which way is decided by how the sensor is mounted, so
north_is_high is a config value checked with test_magnet.py.

The idle point is measured at start-up rather than hardcoded, because it
drifts with temperature and battery voltage. Calibrate with nothing magnetic
near the sensor.

A wrong pole sends the package to the wrong deck, so a reading must repeat
confirm_reads times before it is believed. poll() never blocks.

Wiring: the output must reach the ADC pin directly and stay within 0-3.3 V.
A digital opto-isolator stage would turn it into on/off and lose the pole.
"""

from machine import ADC, Pin
from utime import ticks_ms, ticks_diff

NORTH = "north"
SOUTH = "south"


class MagnetSensor:
    def __init__(self, adc_pin, deadband, north_is_high, poll_ms,
                 confirm_reads):
        self._adc = ADC(Pin(adc_pin))
        self._deadband = deadband
        self._north_is_high = north_is_high
        self._poll_ms = poll_ms
        self._confirm_reads = confirm_reads

        self.centre = 65535 // 2    # replaced by calibrate()
        self.raw = self.centre
        self.pole = None            # the confirmed pole, or None

        self._candidate = None
        self._count = 0
        self._last_poll_ms = ticks_ms()

    def calibrate(self, samples):
        """Measure the idle output. Call with no magnet near the sensor."""
        total = 0
        for _ in range(samples):
            total += self._adc.read_u16()
        self.centre = total // samples
        return self.centre

    def _filtered(self):
        """Median of three, so a motor switching spike is discarded."""
        a = self._adc.read_u16()
        b = self._adc.read_u16()
        c = self._adc.read_u16()
        if a > b:
            a, b = b, a
        if b > c:
            b = c if c > a else a
        return b

    def read(self):
        """NORTH, SOUTH or None for one filtered reading, unconfirmed."""
        self.raw = self._filtered()
        offset = self.raw - self.centre
        if offset >= self._deadband:
            return NORTH if self._north_is_high else SOUTH
        if offset <= -self._deadband:
            return SOUTH if self._north_is_high else NORTH
        return None

    def reset(self):
        """Forget the current reading, ready for the next package."""
        self._candidate = None
        self._count = 0
        self.pole = None
        self._last_poll_ms = ticks_ms()

    def poll(self):
        """Return the pole on the pass that newly confirms it, else None.

        Call every main-loop pass; it samples at most every poll_ms.
        """
        now = ticks_ms()
        if ticks_diff(now, self._last_poll_ms) < self._poll_ms:
            return None
        self._last_poll_ms = now

        reading = self.read()
        if reading is None:
            # No magnet in range - drop any part-confirmed reading.
            self._candidate = None
            self._count = 0
            self.pole = None
            return None

        if reading != self._candidate:
            self._candidate = reading
            self._count = 1
            return None

        self._count += 1
        if self._count < self._confirm_reads or self.pole == reading:
            return None

        self.pole = reading
        return reading
