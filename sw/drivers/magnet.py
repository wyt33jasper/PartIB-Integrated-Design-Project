"""Tell a package's north pole from its south pole.

This is the only measurement the delivery depends on, and since the QR
reader was removed it is the only way the AGV identifies a package at all -
so a failure here is not a degraded delivery, it is no delivery. From the
task spec:

    "It must be able to automatically recognise packages via magnetic field
     measurement, thus identifying the storage location or delivery bay"

    "The boxes with north pole magnets need to be delivered to the top racks,
     those with south pole magnets to the bottom racks"

and the scoring gives +15 a pallet for a package identified by pole and
delivered correctly, against +5 for the wrong depot. Getting the pole wrong
costs two thirds of the score for that package, so the reading is confirmed
rather than trusted on sight: MAGNET_CONFIRM_READS identical readings are
required, a transient cannot flip a pole that is already reported, and both
sensors reading high at once is rejected rather than guessed at.

With nothing left to cross-check it against, sw/test_magnet.py on the bench
is the whole verification - it settles which sensor is which and what the
working distance is. Repeat it with several packages: the spec warns the
blocks vary, and if their magnets vary too then so does the distance at
which a pole becomes readable. That distance is also the grip trigger,
because neither sensor reads high until the magnet is in range, so "a pole
is readable" and "close enough to grip" are the same signal.

There are two ways to read the pole, and which one is available is decided
by the wiring rather than by preference:

  DigitalHallPair  two unipolar Hall sensors facing opposite ways. One reads
                   high for a north pole, the other for a south pole, and
                   neither reads high with no package. Two plain GPIO pins.

  AnalogueHall     one ratiometric Hall sensor on an ADC pin, idling near
                   half supply and swinging either side. Better, because it
                   gives field strength rather than presence, so a weak or
                   distant magnet is distinguishable from a strong one. It
                   needs one of the Pico's three ADC pins.

The M1 wiring leaves no ADC pin free - the beacon is on GP26 and the turn
trackers on GP27 and GP28 - so the digital pair is what the robot can
actually use today. Freeing an ADC is a matter of moving one turn tracker to
a plain GPIO; AnalogueHall is kept ready for that.

MagnetSensor wraps either one with the part that matters: a pole must read
the same several times running before it is believed. Reading absence is
deliberately not an event. poll() reports a pole once, when it is newly
confirmed; an empty gripper just clears the state so the next package starts
fresh.
"""

from machine import Pin
from utime import ticks_ms, ticks_diff

NORTH = "north"
SOUTH = "south"

_ADC_MAX = 65535


class DigitalHallPair:
    """Two unipolar Hall sensors, one per pole.

    Unambiguous by construction: each sensor only responds to the pole it
    faces, so there is no threshold to tune and nothing to calibrate.

    Both reading high at once is not a real package - it means the sensors
    are mis-wired or something else is in the field - so it is reported as
    no reading rather than guessed at.
    """

    def __init__(self, north_pin, south_pin, active_high=True):
        # Pull to the inactive level so an unplugged sensor reads "no
        # magnet" rather than floating.
        pull = Pin.PULL_DOWN if active_high else Pin.PULL_UP
        self._north = Pin(north_pin, Pin.IN, pull)
        self._south = Pin(south_pin, Pin.IN, pull)
        self._active = 1 if active_high else 0
        self.raw = (0, 0)

    def calibrate(self, samples=0):
        """Nothing to calibrate on a digital sensor."""
        return None

    def read(self):
        n = self._north.value() == self._active
        s = self._south.value() == self._active
        self.raw = (1 if n else 0, 1 if s else 0)
        if n and not s:
            return NORTH
        if s and not n:
            return SOUTH
        return None


class AnalogueHall:
    """One ratiometric Hall sensor on an ADC pin.

    The idle point is measured at startup, not hardcoded: the quiescent
    output drifts with temperature and with battery voltage across a five
    minute run, and a fixed centre would slowly bias the deadband towards
    one pole.

    north_is_high exists because which way the sensor faces decides whether
    north reads above or below idle. That is a mounting detail, not
    something to assume.
    """

    def __init__(self, adc_pin, deadband, north_is_high=True, centre=None):
        if adc_pin is None:
            raise ValueError(
                "No ADC pin assigned. Free GP26, GP27 or GP28 first, or use "
                "DigitalHallPair.")
        from machine import ADC
        self._adc = ADC(Pin(adc_pin))
        self._deadband = deadband
        self._north_is_high = north_is_high
        self._centre = _ADC_MAX // 2 if centre is None else centre
        self.raw = self._centre

    @property
    def centre(self):
        return self._centre

    def calibrate(self, samples=32):
        """Measure the idle output with no package in the gripper."""
        total = 0
        for _ in range(samples):
            total += self._adc.read_u16()
        self._centre = total // samples
        return self._centre

    def _filtered(self):
        """Median of three samples.

        A Hall reading occasionally jumps on a motor switching transient.
        The mean would carry that spike into the result; the median
        discards it.
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

    def read(self):
        self.raw = self._filtered()
        return self.classify(self.raw)


def probe_from_config(config):
    """Build whichever probe the wiring supports, as config.py selects."""
    if config.MAGNET_MODE == "digital":
        return DigitalHallPair(config.MAGNET_NORTH_PIN,
                               config.MAGNET_SOUTH_PIN,
                               active_high=config.MAGNET_ACTIVE_HIGH)
    if config.MAGNET_MODE == "analogue":
        return AnalogueHall(config.MAGNET_ADC_PIN,
                            config.MAGNET_DEADBAND,
                            north_is_high=config.MAGNET_NORTH_IS_HIGH)
    raise ValueError("Unknown MAGNET_MODE: {}".format(config.MAGNET_MODE))


class MagnetSensor:
    """Confirm a pole before believing it, whichever probe is fitted."""

    def __init__(self, probe, poll_ms=20, confirm_reads=3):
        self._probe = probe
        self._poll_ms = poll_ms
        self._confirm_reads = confirm_reads

        self._last_due_ms = ticks_ms()
        self._candidate = None
        self._count = 0
        self.pole = None            # the confirmed pole, or None

    @property
    def raw(self):
        """The probe's last reading, for bench tests."""
        return self._probe.raw

    @property
    def confirmations(self):
        return self._count

    def calibrate(self, samples=32):
        return self._probe.calibrate(samples)

    def reset(self):
        """Forget the current reading, ready for the next package."""
        self._candidate = None
        self._count = 0
        self.pole = None
        self._last_due_ms = ticks_ms()

    def poll(self):
        """Return a pole on the pass that newly confirms it, else None.

        Must be called every main-loop pass. Never blocks.
        """
        now = ticks_ms()
        if ticks_diff(now, self._last_due_ms) < self._poll_ms:
            return None
        self._last_due_ms = now

        reading = self._probe.read()

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
