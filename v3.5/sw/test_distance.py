"""Bench test for the rear distance sensor. Nothing moves.

Prints the rear reading in mm, twice a second. Use it to set BAY_FULL_MM.

  Wiring      A "WARNING: no rear distance sensor" at start-up means the bus
              sees nothing at 0x29 - check SDA/SCL and DISTANCE_REAR. Wave a
              hand behind the AGV and make sure the number changes.

  Threshold   Back the AGV up to a shelf bay the way it will approach in a
              run. Note the reading with a package in the bay and with the
              bay empty. BAY_FULL_MM goes between the two, nearer the
              package reading. About 8190 means nothing in range.
"""

from utime import sleep_ms

import config
from drivers.distance import build_rear


def test_distance():
    rear = build_rear(config)
    print("BAY_FULL_MM = {} - at or under this counts as a package"
          .format(config.BAY_FULL_MM))
    while True:
        # read() only returns a value when a new measurement has landed;
        # last_mm holds the newest one either way.
        rear.read()
        print("  rear {:>5} mm".format(str(rear.last_mm)))
        sleep_ms(500)


if __name__ == "__main__":
    test_distance()
