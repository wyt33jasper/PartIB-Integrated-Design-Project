"""Bench test for the Hall sensor and the red/green pole lamps.

Run this with the AGV on the bench and a package magnet in hand. It prints
the live reading and the pole it classifies, and drives the lamps.

With MAGNET_MODE = "digital" (what the M1 wiring supports) it shows both
sensor lines, so three things can be settled:

  Which is which    Face the sensor pair with a known north pole. If it
                    prints "south", the two sensors are swapped - change
                    MAGNET_NORTH_PIN and MAGNET_SOUTH_PIN over.
  Both high         Means the sensors are mis-wired or something else is in
                    the field. The reading is rejected, not guessed at.
  Working distance  Move the magnet away until the pole drops out. That is
                    how close the gripper has to bring a package.

With MAGNET_MODE = "analogue" it calibrates the idle point first, then shows
the raw ADC reading and its offset, so the deadband can be set above the
noise floor and below what a real package produces.
"""

from utime import sleep_ms

import config
from drivers.indicators import PoleIndicator
from drivers.magnet import MagnetSensor, probe_from_config


def test_magnet():
    probe = probe_from_config(config)
    sensor = MagnetSensor(probe,
                          poll_ms=config.MAGNET_POLL_MS,
                          confirm_reads=config.MAGNET_CONFIRM_READS)
    indicator = PoleIndicator(config.RED_LED_PIN, config.GREEN_LED_PIN)

    print("magnet mode: {}".format(config.MAGNET_MODE))
    if config.MAGNET_MODE == "analogue":
        print("keep magnets away - calibrating the idle point")
        sleep_ms(500)
        centre = probe.calibrate(config.MAGNET_CALIBRATE_SAMPLES)
        print("calibrated: idle = {}, deadband = +/-{}".format(
            centre, config.MAGNET_DEADBAND))
    else:
        centre = 0
        print("north sensor on GP{}, south sensor on GP{}".format(
            config.MAGNET_NORTH_PIN, config.MAGNET_SOUTH_PIN))

    print("bring a package magnet towards the sensor")
    print()
    print("  reading          pole")

    last = None
    while True:
        reported = sensor.poll()
        if reported is not None:
            print("  >>> confirmed {}".format(reported))

        # Keep the lamps matching the confirmed pole on every pass. show()
        # is idempotent, so this needs no change tracking.
        indicator.show(sensor.pole)

        if config.MAGNET_MODE == "analogue":
            offset = sensor.raw - centre
            # Bucket the value, or the shell scrolls unreadably.
            shown = (offset // 250, sensor.pole)
            text = "{:5d} ({:+6d})".format(sensor.raw, offset)
        else:
            shown = (sensor.raw, sensor.pole)
            text = "N={} S={}     ".format(sensor.raw[0], sensor.raw[1])

        if shown != last:
            last = shown
            print("  {}   {}".format(text, sensor.pole or "-"))

        sleep_ms(config.LOOP_PERIOD_MS)


if __name__ == "__main__":
    test_magnet()
