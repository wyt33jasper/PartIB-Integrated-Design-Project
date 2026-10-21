"""Side-facing rangefinders, for deciding whether a bay is already occupied.

The spec says to "only deliver if the location is not already occupied", and
the AGV has no memory of earlier runs to consult - it may be started with
packages already on the racks. So occupancy has to be measured, and measuring
it from the corridor, before committing to a turn, is what makes the delivery
leg cheap: an occupied bay costs nothing but the width of the junction.

One sensor on each side, pointed at the bay mouths. An occupied bay returns
the face of the box, a short way off. An empty one returns the back of the
rack, much further, or nothing at all.

    corridor ----+-------------+-------------+----
                 |             |             |
              [ box ]       [     ]       [ box ]
             occupied        empty        occupied
             short range    long range    short range

Two sensor types are supported because the choice is a wiring question:

  ToFRanger       a VL53L0X or TMF8x01 style device on I2C, returning a
                  distance in millimetres. Reports how far away the target
                  is, so the threshold is a tunable number rather than a
                  property of the part.

  DigitalRanger   a sensor in IO mode - the STU-22L, for instance - whose
                  single line goes active when a target is closer than its
                  own fixed trip point. One plain GPIO, nothing on the bus,
                  but the trip distance is set in hardware.

Both are read through the same confirm ladder as the magnet sensor: a bay is
not called empty on the strength of one reading. A wrong answer either stacks
a package on top of another or skips a free bay, and both cost marks.

Two identical I2C parts share one address, so a pair of VL53L0Xs cannot sit
on the same bus without XSHUT sequencing. Config puts them on separate buses
instead - I2C0 and I2C1 - which needs no boot-time address juggling. A
VL53L0X (0x29) and a TMF8x01 (0x41) would share a bus happily.
"""

from utime import ticks_ms, ticks_diff

LEFT = "left"
RIGHT = "right"
SIDES = (LEFT, RIGHT)

OCCUPIED = True
EMPTY = False
UNKNOWN = None


class ToFRanger:
    """A time-of-flight device that reports millimetres.

    `device` is anything with read() returning a distance in mm - the
    VL53L0X class in libs/, or the TMF8x01 wrapper below.

    min_mm rejects readings below the sensor's own crosstalk floor: a ToF
    module behind a window can report a few millimetres with nothing in
    front of it at all, and treating that as a box would skip every bay.
    """

    def __init__(self, device, occupied_mm, min_mm, max_mm):
        self._device = device
        self._occupied_mm = occupied_mm
        self._min_mm = min_mm
        self._max_mm = max_mm
        self.raw = None

    def near(self):
        """True if a box is in the bay, False if not, None if unreadable."""
        try:
            mm = self._device.read()
        except OSError:
            # An I2C read can fail on a loose connector mid-run. Report it
            # as unreadable rather than letting it stop the AGV.
            self.raw = None
            return UNKNOWN

        self.raw = mm
        if mm is None:
            return UNKNOWN
        if mm < self._min_mm:
            return UNKNOWN              # below the crosstalk floor
        if mm > self._max_mm:
            return EMPTY                # nothing within range of the bay
        return mm <= self._occupied_mm


class TMF8x01Device:
    """read() wrapper so a TMF8x01 looks like the VL53L0X class."""

    def __init__(self, sensor):
        self._sensor = sensor

    def read(self):
        return self._sensor.get_distance_mm()


class DigitalRanger:
    """A proximity sensor in IO mode: one line, active when a target is near.

    The trip distance is fixed in the sensor, so there is nothing to tune
    here - but equally nothing to adjust if it trips at the wrong range.
    """

    def __init__(self, pin_no, active_high):
        from machine import Pin
        # Pull to the inactive level, so an unplugged sensor reads "nothing
        # there" rather than floating. For a bay scan that means the AGV
        # tries the bay, which the gripper will then fail on visibly,
        # instead of silently skipping every bay on the rack.
        pull = Pin.PULL_DOWN if active_high else Pin.PULL_UP
        self._pin = Pin(pin_no, Pin.IN, pull)
        self._active = 1 if active_high else 0
        self.raw = None

    def near(self):
        self.raw = self._pin.value()
        return self.raw == self._active


class NullRanger:
    """Stands in for a sensor that is not fitted.

    Reports UNKNOWN rather than EMPTY, so the scan policy decides what to do
    about a missing sensor instead of this class quietly asserting that
    every bay on the rack is free.
    """

    def __init__(self):
        self.raw = None

    def near(self):
        return UNKNOWN


class BayScanner:
    """The two side rangers, sampled across a measured look at one bay.

    The hard part is not the measurement but when to take it. The sensor is
    level with a bay mouth only briefly - the mouth is at the junction, and
    the AGV is past it a moment later - and for the whole of the drive up
    the corridor it is looking at the rack frame instead. A reading that
    simply tracks the latest confirmed value therefore answers with
    whatever the corridor looked like, which is "empty", whether the bay is
    full or not.

    So a scan is a bounded look rather than a running average. arm() starts
    one, poll() gathers samples for as long as the main loop keeps the AGV
    beside the mouth, and occupied() reports what that look found:

        armed    at the junction, before the AGV creeps forward
        polled   while it brings the sensor level with the bay mouth
        read     once, to decide whether to turn in

    Counting near readings rather than averaging distances is deliberate.
    An occupied bay returns the box face from part of the look and the rack
    frame from the rest, so the mean sits between the two and means
    nothing; what matters is whether anything close was seen at all.

    confirm_reads of them are needed, so one glitch cannot condemn a free
    bay. That sets a floor on how long the look has to last:

        BAY_SCAN_ADVANCE_MS >= RANGER_POLL_MS * RANGER_CONFIRM_READS

    Below that there is no time to gather the samples and every bay reads
    UNKNOWN. main() checks this at start-up rather than leaving it to be
    discovered on the table.
    """

    def __init__(self, left, right, poll_ms, confirm_reads):
        self._rangers = {LEFT: left, RIGHT: right}
        self._poll_ms = poll_ms
        self._confirm_reads = confirm_reads

        self._samples = {LEFT: 0, RIGHT: 0}
        self._near = {LEFT: 0, RIGHT: 0}
        self._last_due_ms = ticks_ms()
        self.arm()

    @property
    def fitted(self):
        """False when either side is a NullRanger, for a warning at boot."""
        return not any(isinstance(r, NullRanger)
                       for r in self._rangers.values())

    @property
    def confirm_reads(self):
        return self._confirm_reads

    @property
    def poll_ms(self):
        return self._poll_ms

    def arm(self):
        """Begin a fresh look. Discards everything the last one gathered."""
        for side in SIDES:
            self._samples[side] = 0
            self._near[side] = 0
        self._last_due_ms = ticks_ms()

    # Kept so callers that only want to forget the state can say so.
    reset = arm

    def distance(self, side):
        """The last raw reading on that side, for bench tests."""
        return self._rangers[side].raw

    def samples(self, side):
        """Readable samples gathered in this look."""
        return self._samples[side]

    def near_count(self, side):
        """How many of them found something close."""
        return self._near[side]

    def occupied(self, side):
        """OCCUPIED, EMPTY or UNKNOWN for the bay looked at on that side.

        UNKNOWN means the look gathered too few readable samples to say -
        the sensor is not fitted, or it failed, or it was not given long
        enough. It does not mean the bay is empty, and the mission decides
        separately what to do about it.
        """
        if self._samples[side] < self._confirm_reads:
            return UNKNOWN
        if self._near[side] >= self._confirm_reads:
            return OCCUPIED
        return EMPTY

    def poll(self):
        """Take a sample on each side if poll_ms has elapsed. Never blocks."""
        now = ticks_ms()
        if ticks_diff(now, self._last_due_ms) < self._poll_ms:
            return
        self._last_due_ms = now

        for side in SIDES:
            reading = self._rangers[side].near()
            if reading is UNKNOWN:
                continue            # not a sample either way
            self._samples[side] += 1
            if reading:
                self._near[side] += 1


def _i2c(bus_id, sda, scl, freq):
    from machine import I2C, Pin
    return I2C(id=bus_id, sda=Pin(sda), scl=Pin(scl), freq=freq)


def ranger_from_config(config, side):
    """Build one side's rangefinder as config.py describes it."""
    mode = config.RANGER_MODE
    if mode == "none":
        return NullRanger()

    pins = config.RANGER_PINS[side]

    if mode == "digital":
        return DigitalRanger(pins["pin"],
                             active_high=config.RANGER_ACTIVE_HIGH)

    if mode == "tof":
        from libs.VL53L0X.VL53L0X import VL53L0X
        device = VL53L0X(_i2c(pins["i2c_id"], pins["sda"], pins["scl"],
                              config.RANGER_I2C_FREQ),
                         address=pins.get("address", 0x29))
        device.start()
        return ToFRanger(device,
                         occupied_mm=config.BAY_OCCUPIED_MM,
                         min_mm=config.BAY_MIN_MM,
                         max_mm=config.BAY_MAX_MM)

    raise ValueError("Unknown RANGER_MODE: {}".format(mode))


def scanner_from_config(config):
    """Build the bay scanner, whichever sensors the wiring has."""
    return BayScanner(ranger_from_config(config, LEFT),
                      ranger_from_config(config, RIGHT),
                      poll_ms=config.RANGER_POLL_MS,
                      confirm_reads=config.RANGER_CONFIRM_READS)
