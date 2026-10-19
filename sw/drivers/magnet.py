"""Tell a package's north pole from its south pole.

This is the only measurement the delivery depends on. From the task spec:

    "It must be able to automatically recognise packages via magnetic field
     measurement, thus identifying the storage location or delivery bay"

    "The boxes with north pole magnets need to be delivered to the top racks,
     those with south pole magnets to the bottom racks"

and the scoring gives +15 a pallet for a package identified by pole and
delivered correctly, against +5 for the wrong depot. Getting the pole wrong
costs two thirds of the score for that package, so the reading is confirmed
rather than trusted on sight.

A ratiometric Hall sensor idles near half supply and swings up for one pole
and down for the other. Which way round depends on which face of the sensor
the magnet approaches, so MAGNET_NORTH_IS_HIGH exists rather than an
assumption.

The idle point is measured at startup, not hardcoded. It drifts with
temperature and with battery voltage over a five minute run, and a fixed
constant would slowly bias the deadband towards one pole.

Reading absence is deliberately not an event. poll() reports a pole once,
when it is newly confirmed; an empty gripper just clears the state so the
next package starts fresh.
"""

from machine import ADC, Pin
from utime import ticks_ms, ticks_diff

NORTH = "north"
SOUTH = "south"

_ADC_MAX = 65535


class MagnetSensor:
    def __init__(self, adc_pin, deadband, north_is_high=True,
                 poll_ms=20, confirm_reads=3, centre=None):
        self._adc = ADC(Pin(adc_pin))
        self._deadband = deadband
        self._north_is_high = north_is_high
        self._poll_ms = poll_ms
        self._confirm_reads = confirm_reads

        # Half scale is a reasonable starting point for a ratiometric sensor,
        # but calibrate() should replace it before the run starts.
        self._centre = _ADC_MAX // 2 if centre is None else centre

        self._last_due_ms = ticks_ms()
        self._candidate = None
        self._count = 0
        self.pole = None            # the confirmed pole, or None
        self.raw = self._centre     # last filtered reading, for bench tests

    def calibrate(self, samples=32):
        """Measure the idle output with no package in the gripper.

        Call once at startup, before the first pickup. Blocking is fine here:
        the AGV is stationary and waiting for the start button.
        """
        total = 0
        for _ in range(samples):
            total += self._adc.read_u16()
        self._centre = total // samples
        return self._centre

    @property
    def centre(self):
        return self._centre

    @property
    def confirmations(self):
        return self._count

    def reset(self):
        """Forget the current reading, ready for the next package."""
        self._candidate = None
        self._count = 0
        self.pole = None
        self._last_due_ms = ticks_ms()

    def _read_filtered(self):
        """Median of three samples.

        A Hall reading occasionally jumps on a motor switching transient. The
        mean would carry that spike into the result; the median discards it.
        """
        a = self._adc.read_u16()
        b = self._adc.read_u16()
        c = self._adc.read_u16()
        if a > b:
            a, b = b, a
        if b > c:
            b = c if c > a else a
        return b

    def classify(self, value):
        """Return NORTH, SOUTH or None for one filtered reading."""
        offset = value - self._centre
        if offset >= self._deadband:
            return NORTH if self._north_is_high else SOUTH
        if offset <= -self._deadband:
            return SOUTH if self._north_is_high else NORTH
        return None

    def poll(self):
        """Return a pole on the pass that newly confirms it, else None.

        Must be called every main-loop pass. Never blocks.
        """
        now = ticks_ms()
        if ticks_diff(now, self._last_due_ms) < self._poll_ms:
            return None
        self._last_due_ms = now

        self.raw = self._read_filtered()
        reading = self.classify(self.raw)

        if reading is None:
            # No magnet in range. Drop any partial confirmation so a package
            # being removed cannot complete the count for the next one.
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
