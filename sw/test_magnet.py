"""Bench test for the Hall sensor and the red/green pole lamps.

Run this with the AGV on the bench and a package magnet in hand. It prints
the live ADC reading and the pole it classifies, and drives the lamps, so
three things can be settled at once:

  Which way round   Face the sensor with a known north pole. If it prints
                    "south", flip MAGNET_NORTH_IS_HIGH in config.py.

  Deadband          Watch the "offset" column with no magnet near. That is
                    the noise floor. MAGNET_DEADBAND must sit comfortably
                    above it, and comfortably below the offset a real
                    package produces at its working distance.

  Working distance  Move the magnet away until the pole drops out. That is
                    how close the gripper has to bring a package.

The idle point is calibrated first, so keep magnets away until it says
"calibrated".
"""

from utime import sleep_ms

import config
from drivers.indicators import PoleIndicator
from drivers.magnet import MagnetSensor


def test_magnet():
    sensor = MagnetSensor(config.MAGNET_ADC_PIN,
                          deadband=config.MAGNET_DEADBAND,
                          north_is_high=config.MAGNET_NORTH_IS_HIGH,
                          poll_ms=config.MAGNET_POLL_MS,
                          confirm_reads=config.MAGNET_CONFIRM_READS)
    indicator = PoleIndicator(config.RED_LED_PIN, config.GREEN_LED_PIN)

    print("keep magnets away - calibrating the idle point")
    sleep_ms(500)
    centre = sensor.calibrate(config.MAGNET_CALIBRATE_SAMPLES)
    print("calibrated: idle = {}, deadband = +/-{}".format(
        centre, config.MAGNET_DEADBAND))
    print("bring a package magnet towards the sensor")
    print()
    print("  raw    offset   pole")

    last_pole = None
    last_bucket = None
    while True:
        reported = sensor.poll()
        if reported is not None:
            print("  >>> confirmed {}".format(reported))

        # Keep the lamps matching the confirmed pole on every pass. show()
        # is idempotent, so this needs no change tracking.
        indicator.show(sensor.pole)

        offset = sensor.raw - centre
        # Print on a meaningful change only, or the shell scrolls unreadably.
        bucket = offset // 250
        if bucket != last_bucket or sensor.pole != last_pole:
            last_bucket = bucket
            last_pole = sensor.pole
            print("  {:5d}  {:+6d}   {}".format(
                sensor.raw, offset, sensor.pole or "-"))

        sleep_ms(config.LOOP_PERIOD_MS)


if __name__ == "__main__":
    test_magnet()
