"""Side-facing distance sensors, one per side of the AGV.

Two VL53L0X. Both answer at 0x29, so they sit on separate buses:
I2C0 on GP8/GP9 and I2C1 on GP10/GP11.

read() never blocks. The library's own read() spins for up to a second
waiting for a measurement, which would stall line following. Instead the
sensor is left ranging continuously and read() just asks whether a new
measurement has landed yet.
"""

from machine import I2C, Pin

_ADDRESS = 0x29             # both VL53L0X answer here; hence two buses
_I2C_FREQ = 400000

_INTERRUPT_STATUS = 0x13
_RANGE_STATUS = 0x14
_INTERRUPT_CLEAR = 0x0B
_READY = 0x07


class DistanceSensor:
    def __init__(self, i2c_id, sda_pin, scl_pin):
        from libs.VL53L0X.VL53L0X import VL53L0X
        i2c = I2C(id=i2c_id, sda=Pin(sda_pin), scl=Pin(scl_pin),
                  freq=_I2C_FREQ)
        self._dev = VL53L0X(i2c, address=_ADDRESS)
        self._dev.start()           # continuous, back-to-back ranging
        self.last_mm = None

    def read(self):
        """Distance in mm, or None if no new measurement is ready."""
        try:
            if not self._dev._register(_INTERRUPT_STATUS) & _READY:
                return None
            mm = self._dev._register(_RANGE_STATUS + 10, struct=">H")
            self._dev._register(_INTERRUPT_CLEAR, 0x01)
        except OSError:
            return None             # loose connector - skip this reading
        self.last_mm = mm
        return mm


class NoSensor:
    """Stand-in when a sensor is not fitted. Always reads nothing."""

    last_mm = None

    def read(self):
        return None


def _build_one(side, bus, sda, scl):
    """One sensor, or NoSensor if it does not answer.

    A missing sensor must not stop the robot booting - the bay detector
    then reports every bay full, which is the safe answer.
    """
    try:
        return DistanceSensor(bus, sda, scl)
    except (OSError, RuntimeError) as exc:
        print("WARNING: no %s distance sensor on I2C%d (GP%d/GP%d): %s"
              % (side, bus, sda, scl, exc))
        return NoSensor()


def build_pair(config):
    """(left, right) sensors as config describes them."""
    if not config.DISTANCE_FITTED:
        return NoSensor(), NoSensor()
    return (_build_one("left", *config.DISTANCE_LEFT),
            _build_one("right", *config.DISTANCE_RIGHT))
