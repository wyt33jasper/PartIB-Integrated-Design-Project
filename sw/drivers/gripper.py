"""The lifting and gripping mechanism, driven without blocking.

The linear actuator has no position feedback - the department's own test
notes say so: "we don't know when this has finished without another means".
So a movement is a timed run: drive the actuator for a measured duration,
then cut it. The durations belong in config.py and are measured on the
bench, like every other timing in this robot.

The reason this is not simply a pair of sleeps is the stop button. A grip
and a lift together take several seconds; sleeping through them would leave
the AGV holding a package with its safety control unread. So a movement is
started, and update() cuts the motor when the deadline passes. The main loop
keeps running throughout.

    grip = Gripper(...)
    grip.close()
    while grip.busy:
        grip.update()
        ...stop button, beacon, magnet sensor all still polled...
"""

from machine import Pin, PWM
from utime import ticks_ms, ticks_diff

PWM_FREQ_HZ = 1000
_FULL_SCALE = 65535

OPEN = "open"
CLOSED = "closed"


class Gripper:
    """One linear actuator channel on a TB6612FNG driver."""

    # Which direction line value extends the actuator. The department's
    # actuator test uses dir=0 to extend.
    _EXTEND = 0
    _RETRACT = 1

    def __init__(self, dir_pin, pwm_pin, close_ms, open_ms, speed,
                 closed_is_extended=True):
        self._dir = Pin(dir_pin, Pin.OUT)
        self._pwm = PWM(Pin(pwm_pin))
        self._pwm.freq(PWM_FREQ_HZ)
        self._pwm.duty_u16(0)

        self._close_ms = close_ms
        self._open_ms = open_ms
        self._speed = speed
        self._closed_is_extended = closed_is_extended

        self._deadline_ms = None
        self._target = OPEN
        self.position = OPEN        # assumed at power-up; see home()

    @property
    def busy(self):
        return self._deadline_ms is not None

    @property
    def holding(self):
        return self.position == CLOSED and not self.busy

    def _run(self, extend, duration_ms, target):
        self._dir.value(self._EXTEND if extend else self._RETRACT)
        self._pwm.duty_u16(int(_FULL_SCALE * self._speed / 100))
        self._deadline_ms = ticks_ms() + duration_ms
        self._target = target

    def close(self):
        """Begin gripping. Returns immediately; poll update() until not busy."""
        self._run(self._closed_is_extended, self._close_ms, CLOSED)

    def open(self):
        """Begin releasing. Returns immediately."""
        self._run(not self._closed_is_extended, self._open_ms, OPEN)

    def home(self):
        """Drive to the open end stop, to establish a known position.

        Worth calling once at start-up: the actuator keeps its position
        through a power cycle, so "assumed open" can be wrong.
        """
        self.open()

    def update(self):
        """Cut the actuator when its movement time is up. Call every pass."""
        if self._deadline_ms is None:
            return
        if ticks_diff(ticks_ms(), self._deadline_ms) >= 0:
            self.stop()
            self.position = self._target

    def stop(self):
        """Cut drive. Safe to call at any time, repeatedly."""
        self._pwm.duty_u16(0)
        self._deadline_ms = None


class NullGripper:
    """Stands in while the mechanism is not wired up.

    Movements complete instantly, so the whole mission can be driven on a
    chassis with no gripper fitted - which is how the navigation gets tested
    before the mechanical side is finished.
    """

    def __init__(self):
        self.position = OPEN
        self.fitted = False

    busy = False

    @property
    def holding(self):
        return self.position == CLOSED

    def close(self):
        self.position = CLOSED

    def open(self):
        self.position = OPEN

    def home(self):
        self.open()

    def update(self):
        pass

    def stop(self):
        pass


def gripper_from_config(config):
    if not config.GRIPPER_FITTED:
        return NullGripper()
    gripper = Gripper(config.GRIPPER_DIR_PIN, config.GRIPPER_PWM_PIN,
                      close_ms=config.GRIPPER_CLOSE_MS,
                      open_ms=config.GRIPPER_OPEN_MS,
                      speed=config.GRIPPER_SPEED,
                      closed_is_extended=config.GRIPPER_CLOSED_IS_EXTENDED)
    gripper.fitted = True
    return gripper
