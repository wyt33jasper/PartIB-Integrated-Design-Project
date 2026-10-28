"""On-robot test for the reverse-up-to-the-bay manoeuvre.

Stand the AGV on the corridor line at a shelf junction, with the bay on the
side set by SIDE below, and press the button. It runs the real manoeuvre from
main.py once: rotate its back to the bay, reverse up to it, measure, and
either face into the bay (empty) or return to the corridor heading (full).
Nothing is delivered and the mission is not involved.

Tune in this order, re-running after each change:

  1. BAY_FULL_MM        with test_distance.py, before anything moves.
  2. BAY_FACE_AWAY_MS   until the first rotation is a square 90 deg.
  3. BAY_APPROACH_MS    until the reverse stops at the bay mouth, close
                        enough for the sensor but not touching the rack.
  4. BAY_SPIN_MS        until, on an empty bay, the AGV ends up square on
                        the bay's own line. "no line after the bay spin"
                        means this one is still wrong.

The stop button works throughout.
"""

import config
from drivers.button import Button
from drivers.distance import build_rear
from drivers.drive import Drive
from drivers.indicators import FlashingLamp
from drivers.line_sensor import LineSensorArray
from nav.bay import BayDetector, name as bay_name
from main import _probe_bay

SIDE = "left"           # which side of the AGV the bay is on: "left"/"right"


def test_bay():
    button = Button(config.BUTTON_PIN,
                    active_high=config.BUTTON_ACTIVE_HIGH,
                    debounce_ms=config.BUTTON_DEBOUNCE_MS)
    beacon = FlashingLamp(config.AMBER_LED_PIN, config.AMBER_FLASH_MS)
    drive = Drive(config.LEFT_MOTOR_DIR_PIN, config.LEFT_MOTOR_PWM_PIN,
                  config.RIGHT_MOTOR_DIR_PIN, config.RIGHT_MOTOR_PWM_PIN)
    sensors = LineSensorArray(
        config.LINE_TRACKER_L_PIN,
        config.LINE_TRACKER_R_PIN,
        config.TURN_TRACKER_L_PIN if config.TURN_TRACKERS_FITTED else None,
        config.TURN_TRACKER_R_PIN if config.TURN_TRACKERS_FITTED else None,
        white_is_high=config.LINE_WHITE_IS_HIGH)
    detector = BayDetector(build_rear(config),
                           full_mm=config.BAY_FULL_MM,
                           near_reads=config.BAY_NEAR_READS,
                           confirm_reads=config.BAY_CONFIRM_READS)

    print("bay on the {}, BAY_FULL_MM = {}".format(SIDE, config.BAY_FULL_MM))
    print("press the button to probe the bay")
    button.wait_for_press()
    beacon.on()
    try:
        verdict = _probe_bay(button, beacon, drive, sensors, detector,
                             "test_BAY", SIDE)
        if verdict is None:
            print("stopped")
        else:
            print("verdict: {} at {} mm - bay_is_empty() = {}".format(
                bay_name(verdict), detector.last_mm, detector.bay_is_empty()))
    finally:
        drive.stop()
        beacon.off()


if __name__ == "__main__":
    test_bay()
