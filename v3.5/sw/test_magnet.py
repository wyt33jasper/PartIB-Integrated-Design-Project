"""Bench test for the analogue Hall sensor and the red/green pole lamps.

Run with the motors unplugged and a package in hand. It calibrates the idle
point (keep magnets away for the first second), then prints the raw ADC
reading, its offset from idle, and the confirmed pole, and drives the lamps.

  Direction   Hold a known north face at the sensor. If it prints "south",
              flip MAGNET_NORTH_IS_HIGH in config.py.

  Deadband    Note the offset with no magnet (noise) and with a package held
              where the gripper holds it (signal). MAGNET_DEADBAND goes
              between them. Repeat with several packages - their magnets
              vary - and with the drive motors running, which adds noise.
"""

from utime import sleep_ms

import config
from drivers.indicators import PoleIndicator
from drivers.magnet import MagnetSensor


def test_magnet():
    magnet = MagnetSensor(config.MAGNET_ADC_PIN,
                          deadband=config.MAGNET_DEADBAND,
                          north_is_high=config.MAGNET_NORTH_IS_HIGH,
                          poll_ms=config.MAGNET_POLL_MS,
                          confirm_reads=config.MAGNET_CONFIRM_READS)
    poles = PoleIndicator(config.RED_LED_PIN, config.GREEN_LED_PIN)

    print("keep magnets away - calibrating")
    sleep_ms(1000)
    centre = magnet.calibrate(config.MAGNET_CALIBRATE_SAMPLES)
    print("idle = {}, deadband = +/-{}".format(centre, config.MAGNET_DEADBAND))
    print("bring a package magnet to the sensor")
    print("    raw   offset   pole")

    last = None
    while True:
        confirmed = magnet.poll()
        if confirmed is not None:
            print("  >>> confirmed {}".format(confirmed))
        poles.show(magnet.pole)

        offset = magnet.raw - centre
        shown = (offset // 250, magnet.pole)    # bucketed, or the shell floods
        if shown != last:
            last = shown
            print("  {:5d}  {:+6d}   {}".format(magnet.raw, offset,
                                               magnet.pole or "-"))
        sleep_ms(config.LOOP_PERIOD_MS)


if __name__ == "__main__":
    test_magnet()
