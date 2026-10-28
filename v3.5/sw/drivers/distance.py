"""One VL53L0X distance sensor looking straight back out of the AGV.

It sits at the rear because a side-facing sensor's field of view is far too
wide to tell one bay from its neighbours: at corridor range it sees the
rack's own dividers as packages. Reversing up to a bay instead puts the
sensor close enough that its cone only covers the one bay behind it.

read() never blocks. The library's own read() spins for up to a second
waiting for a measurement, which would stall line following and the stop
button. Instead the sensor ranges continuously and read() only asks whether
a new measurement has landed - the same registers the library uses
(libs/VL53L0X/VL53L0X.py read()), without the wait.
"""

from machine import I2C, Pin

_ADDRESS = 0x29
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
    """Stand-in for a sensor that is not fitted. Never has a reading."""

    last_mm = None

    def read(self):
        return None


def build_rear(config):
    """The rear sensor, or NoSensor if it is not fitted or does not answer.

    A missing sensor must not stop the robot booting. Every bay then reads
    full, which is the safe answer: bays are skipped, nothing is stacked.
    """
    if not config.DISTANCE_FITTED:
        return NoSensor()
    bus, sda, scl = config.DISTANCE_REAR
    try:
        return DistanceSensor(bus, sda, scl)
    except (OSError, RuntimeError) as exc:
        print("WARNING: no rear distance sensor on I2C{} (GP{}/GP{}): {}"
              .format(bus, sda, scl, exc))
        return NoSensor()
