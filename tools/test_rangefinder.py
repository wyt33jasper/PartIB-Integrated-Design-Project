"""Host-side tests for the bay rangefinders.

    python3 tools/test_rangefinder.py

The VL53L0X driver in sw/libs is the department's and is tested on the
bench by sw/test_rangefinder.py. What is tested here is the code wrapped
around it: that a read never blocks, that the three distance thresholds
classify as intended, and that a scan only ever answers from samples taken
during its own look.

The fake sensor below stands in for the real one by implementing just the
two things VL53L0XDevice touches - start() and _register().
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))
sys.path.insert(0, os.path.join(REPO, "sw"))

import utime                                             # noqa: E402
import config                                            # noqa: E402
from drivers.rangefinder import (BayScanner, DigitalRanger,  # noqa: E402
                                 EMPTY, LEFT, NullRanger, OCCUPIED, RIGHT,
                                 ToFRanger, UNKNOWN, VL53L0XDevice,
                                 scanner_from_config)
from machine import Pin                                  # noqa: E402

failures = []


def check(what, got, want):
    ok = got == want
    if not ok:
        failures.append("{} (got {!r}, wanted {!r})".format(what, got, want))
    print("  {}  {:<52} {!r}".format("ok  " if ok else "FAIL", what, got))


class FakeVL53L0X:
    """Implements only what VL53L0XDevice touches."""

    _INTERRUPT_STATUS = 0x13
    _RANGE_STATUS = 0x14
    _INTERRUPT_CLEAR = 0x0B

    def __init__(self):
        self.started = False
        self.ready = False          # has a new measurement landed?
        self.mm = 0
        self.reads = 0
        self.clears = 0
        self.slept_ms = 0           # must stay zero: a read cannot block

    def start(self, period=0):
        self.started = True

    def _register(self, register, value=None, struct="B"):
        self.reads += 1
        if value is not None:
            if register == self._INTERRUPT_CLEAR:
                self.clears += 1
                self.ready = False
            return None
        if register == self._INTERRUPT_STATUS:
            return 0x07 if self.ready else 0x00
        if register == self._RANGE_STATUS + 10:
            return self.mm
        raise AssertionError("unexpected register 0x%02x" % register)


def ranger(sensor, **kw):
    opts = dict(occupied_mm=config.BAY_OCCUPIED_MM,
                min_mm=config.BAY_MIN_MM,
                max_mm=config.BAY_MAX_MM)
    opts.update(kw)
    return ToFRanger(VL53L0XDevice(sensor), **opts)


def main():
    print("the sensor is left ranging continuously")
    fake = FakeVL53L0X()
    device = VL53L0XDevice(fake)
    check("start() called once, at construction", fake.started, True)

    print("\na read never waits for a measurement")
    fake.ready = False
    before = fake.reads
    check("nothing ready yet reports no sample", device.read(), None)
    check("and it cost one register read", fake.reads - before, 1)
    check("it did not sleep", fake.slept_ms, 0)

    fake.ready, fake.mm = True, 137
    check("a measurement is reported", device.read(), 137)
    check("and the interrupt is cleared", fake.clears, 1)
    check("so the next poll finds nothing new", device.read(), None)

    print("\nthe three distance thresholds")
    fake = FakeVL53L0X()
    probe = ranger(fake)
    occupied, floor, ceiling = (config.BAY_OCCUPIED_MM, config.BAY_MIN_MM,
                                config.BAY_MAX_MM)

    def at(mm):
        fake.ready, fake.mm = True, mm
        return probe.near()

    check("below the crosstalk floor is rejected", at(floor - 1), UNKNOWN)
    check("at the floor is a real reading", at(floor), EMPTY if floor > occupied
          else OCCUPIED)
    check("a package face is near", at(occupied - 10), OCCUPIED)
    check("exactly on the threshold is near", at(occupied), OCCUPIED)
    check("just past it is not", at(occupied + 1), EMPTY)
    check("the rack behind an empty bay is not", at(ceiling - 1), EMPTY)
    check("out of range entirely is empty", at(8190), EMPTY)
    fake.ready = False
    check("no measurement is not a reading", probe.near(), UNKNOWN)

    print("\nan I2C failure mid-run is survivable")
    class Broken:
        raw = None

        def read(self):
            raise OSError("bus error")
    broken = ToFRanger(Broken(), occupied_mm=occupied, min_mm=floor,
                       max_mm=ceiling)
    check("reported as unreadable, not raised", broken.near(), UNKNOWN)

    print("\na scan answers only from its own look")
    left, right = FakeVL53L0X(), FakeVL53L0X()
    scanner = BayScanner(ranger(left), ranger(right),
                         poll_ms=config.RANGER_POLL_MS,
                         confirm_reads=config.RANGER_CONFIRM_READS)

    def sample(l_mm, r_mm, times=1):
        for _ in range(times):
            left.ready, left.mm = True, l_mm
            right.ready, right.mm = True, r_mm
            utime.advance(config.RANGER_POLL_MS)
            scanner.poll()

    check("before any samples, nothing is known",
          scanner.occupied(LEFT), UNKNOWN)
    sample(60, 900, times=config.RANGER_CONFIRM_READS - 1)
    check("one sample short, still unknown", scanner.occupied(LEFT), UNKNOWN)
    sample(60, 900)
    check("the last sample settles it", scanner.occupied(LEFT), OCCUPIED)
    check("and the other side independently", scanner.occupied(RIGHT), EMPTY)

    print("\narming discards the previous bay's evidence")
    scanner.arm()
    check("left forgotten", scanner.occupied(LEFT), UNKNOWN)
    check("right forgotten", scanner.occupied(RIGHT), UNKNOWN)
    check("sample counts cleared", (scanner.samples(LEFT),
                                    scanner.near_count(LEFT)), (0, 0))

    print("\na stale reading from the corridor cannot answer for a bay")
    # Drive past open corridor - far readings - then arm at the junction and
    # look into a bay that is full. The corridor must not have voted.
    sample(900, 900, times=10)
    check("the corridor reads empty", scanner.occupied(LEFT), EMPTY)
    scanner.arm()
    sample(60, 60, times=config.RANGER_CONFIRM_READS)
    check("the bay reads occupied", scanner.occupied(LEFT), OCCUPIED)

    print("\nthe scan window is long enough to gather the samples")
    needed = config.RANGER_POLL_MS * config.RANGER_CONFIRM_READS
    check("BAY_SCAN_ADVANCE_MS >= {} ms".format(needed),
          config.BAY_SCAN_ADVANCE_MS >= needed, True)
    check("RANGER_POLL_MS exceeds one VL53L0X measurement (~30 ms)",
          config.RANGER_POLL_MS >= 30, True)

    print("\nreadings the sensor could not make are not votes")
    left2, right2 = FakeVL53L0X(), FakeVL53L0X()
    scanner2 = BayScanner(ranger(left2), ranger(right2),
                          poll_ms=config.RANGER_POLL_MS,
                          confirm_reads=config.RANGER_CONFIRM_READS)
    for _ in range(10):
        left2.ready = right2.ready = False      # no measurement ever lands
        utime.advance(config.RANGER_POLL_MS)
        scanner2.poll()
    check("ten empty polls gather no samples", scanner2.samples(LEFT), 0)
    check("so the bay stays unknown", scanner2.occupied(LEFT), UNKNOWN)

    print("\nconfig puts a VL53L0X on each side, one per bus")
    check("mode is tof", config.RANGER_MODE, "tof")
    buses = {side: config.RANGER_PINS[side]["i2c_id"] for side in (LEFT, RIGHT)}
    check("left and right are on different buses",
          buses[LEFT] != buses[RIGHT], True)
    check("both at the VL53L0X address 0x29",
          [config.RANGER_PINS[s]["address"] for s in (LEFT, RIGHT)],
          [0x29, 0x29])
    pins = []
    for side in (LEFT, RIGHT):
        pins += [config.RANGER_PINS[side]["sda"], config.RANGER_PINS[side]["scl"]]
    check("four distinct I2C pins", len(set(pins)), 4)

    print("\nan unplugged sensor does not stop the robot booting")
    # The host I2C stub raises on every transfer, which is what an absent
    # sensor does, so building from config exercises the fallback.
    scanner3 = scanner_from_config(config)
    check("falls back rather than raising",
          isinstance(scanner3._rangers[LEFT], NullRanger), True)
    check("and says so", scanner3.fitted, False)
    check("every bay then reads unknown", scanner3.occupied(LEFT), UNKNOWN)

    print("\nthe digital backend still works, for a sensor in IO mode")
    pin_no = config.RANGER_PINS[LEFT]["pin"]
    digital = DigitalRanger(pin_no, active_high=True)
    Pin.registry[pin_no].value(1)
    check("active means a package is there", digital.near(), OCCUPIED)
    Pin.registry[pin_no].value(0)
    check("inactive means empty", digital.near(), EMPTY)

    print()
    if failures:
        print("{} FAILED".format(len(failures)))
        for f in failures:
            print("  -", f)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
