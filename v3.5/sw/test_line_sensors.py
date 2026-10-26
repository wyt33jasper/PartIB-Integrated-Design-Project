"""Bench test for the four line sensors.

Run this with the motors unplugged, before trying to follow anything.  It
prints the live state of all four sensors so the team can check three things:

  Polarity   Hold the AGV over white tape.  The display should read WHITE.
             If it reads BLACK, flip LINE_WHITE_IS_HIGH in config.py.

  Wiring     Cover one sensor at a time and confirm the expected column
             changes.  Swapped left and right would make the AGV steer the
             wrong way, which is hard to spot once it is moving.

  Height     The PicoBot guide asks for the minimum and maximum working
             height of each sensor.  Raise and lower the chassis over the
             tape and note where each sensor stops switching cleanly - that
             table is wanted in the technical documentation.

Display, left to right, matches the physical layout:

    turnL  lineL  lineR  turnR
"""

from utime import sleep_ms

import config
from drivers.line_sensor import LineSensorArray


def _label(value):
    return "WHITE" if value else "black"


def test_line_sensors():
    sensors = LineSensorArray(
        config.LINE_TRACKER_L_PIN,
        config.LINE_TRACKER_R_PIN,
        config.TURN_TRACKER_L_PIN if config.TURN_TRACKERS_FITTED else None,
        config.TURN_TRACKER_R_PIN if config.TURN_TRACKERS_FITTED else None,
        white_is_high=config.LINE_WHITE_IS_HIGH)

    if not sensors.has_turn_trackers:
        print("outer pair not fitted - turnL and turnR will always read black")


    print("turnL  lineL  lineR  turnR   | follower would:")
    print("-" * 52)

    last = None
    while True:
        turn_l, line_l, line_r, turn_r = sensors.read_all()
        reading = (turn_l, line_l, line_r, turn_r)

        # Only print on change, otherwise the output scrolls too fast to read.
        if reading != last:
            last = reading

            if turn_l or turn_r:
                action = "junction seen -> turn decider"
            elif line_l and line_r:
                action = "forward"
            elif not line_l and line_r:
                action = "turn right"
            elif line_l and not line_r:
                action = "turn left"
            else:
                action = "reverse (line lost)"

            print(f"{_label(turn_l):>5}  {_label(line_l):>5}  "
                  f"{_label(line_r):>5}  {_label(turn_r):>5}   | {action}")

        sleep_ms(50)


if __name__ == "__main__":
    test_line_sensors()
