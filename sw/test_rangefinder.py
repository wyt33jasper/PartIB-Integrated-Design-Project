"""Bench test for the two side rangefinders, and how to set the threshold.

Run this with the AGV beside the racks. It prints both sides live, with the
verdict each one would give, so the two numbers that decide whether the bay
scan works at all can be measured:

  with a package in the bay    the sensor sees the box face
  with the bay empty           it sees the rack behind, or nothing

BAY_OCCUPIED_MM goes between them. The gap between them is the whole margin
the scan has, so write both down rather than just picking a threshold.

A reading of 8190 or thereabouts means nothing is in range, which is the
clearest possible "empty" - if an empty bay reads that, the scan is easy. If
the rack back is close enough to return a real distance, check the gap is
wide enough to survive the AGV not being perfectly centred in the corridor.

Also worth doing here, before the robot ever drives: hold the AGV still
beside one bay and confirm the left sensor answers for the left bay. Mounted
on the wrong sides, every delivery goes into an occupied bay.
"""

from utime import sleep_ms

import config
from drivers.rangefinder import (EMPTY, LEFT, OCCUPIED, RIGHT,
                                 scanner_from_config)

_VERDICT = {OCCUPIED: "occupied", EMPTY: "empty", None: "unknown"}


def test_rangefinder():
    scanner = scanner_from_config(config)

    print("mode: {}".format(config.RANGER_MODE))
    for side in (LEFT, RIGHT):
        pins = config.RANGER_PINS[side]
        print("  {:5s} I2C{} on GP{}/GP{}, address 0x{:02x}".format(
            side, pins["i2c_id"], pins["sda"], pins["scl"], pins["address"]))
    if not scanner.fitted:
        print("\nNo sensors responded. Check the wiring before going on -")
        print("every bay will read unknown and nothing will be delivered.")
        return

    print("\noccupied at or under {} mm; readings under {} mm are rejected "
          "as\ncrosstalk and over {} mm as nothing in range".format(
              config.BAY_OCCUPIED_MM, config.BAY_MIN_MM, config.BAY_MAX_MM))
    print("\nA verdict looks back {} ms ({} samples) and needs {} of them "
          "to\nagree. Keep that well inside the time one bay takes to "
          "pass.".format(scanner.window_ms, config.RANGER_WINDOW_SAMPLES,
                         scanner.confirm_reads))
    print()
    print("  left            right           verdict (left / right)")

    last = None
    while True:
        scanner.poll()

        left_mm = scanner.distance(LEFT)
        right_mm = scanner.distance(RIGHT)

        # Bucket the distances, or the shell scrolls unreadably.
        shown = (None if left_mm is None else left_mm // 5,
                 None if right_mm is None else right_mm // 5,
                 scanner.occupied(LEFT), scanner.occupied(RIGHT))
        if shown != last:
            last = shown
            print("  {:>8} mm    {:>8} mm    {} / {}".format(
                "-" if left_mm is None else left_mm,
                "-" if right_mm is None else right_mm,
                _VERDICT[scanner.occupied(LEFT)],
                _VERDICT[scanner.occupied(RIGHT)]))

        sleep_ms(config.LOOP_PERIOD_MS)


if __name__ == "__main__":
    test_rangefinder()
