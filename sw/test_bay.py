"""Bench test: drive past the racks and report each bay as empty or full.

Run this on the robot with the AGV on the line beside a rack. It follows
the line and, at every junction, measures the bay on SIDE and prints the
verdict. It never turns - this is the detection chain only.

Use it to set BAY_FULL_MM: put a box in one bay, leave the next empty, and
watch the two distances it prints. The threshold goes between them.

If every bay reads full, either the sensor is not responding or
BAY_TIMEOUT_MS is shorter than the time one bay takes to pass.
"""

from utime import sleep_ms

import config
from drivers.button import Button
from drivers.distance import build_pair
from drivers.drive import Drive
from drivers.indicators import FlashingLamp
from drivers.line_sensor import LineSensorArray
from nav.bay import BayDetector, LEFT, NONE, RIGHT, name
from nav.follower import LineFollower
from nav.junction import NONE as NO_JUNCTION
from nav.junction import JunctionDetector

SIDE = LEFT         # which side the rack is on for this test


def test_bay():
    button = Button(config.BUTTON_PIN,
                    active_high=config.BUTTON_ACTIVE_HIGH,
                    debounce_ms=config.BUTTON_DEBOUNCE_MS)
    beacon = FlashingLamp(config.AMBER_LED_PIN, config.AMBER_FLASH_MS)
    drive = Drive(config.LEFT_MOTOR_DIR_PIN, config.LEFT_MOTOR_PWM_PIN,
                  config.RIGHT_MOTOR_DIR_PIN, config.RIGHT_MOTOR_PWM_PIN,
                  pwm_freq_hz=config.MOTOR_PWM_FREQ_HZ,
                  rotate_scale=config.TURN_ROTATE_SCALE)
    sensors = LineSensorArray(
        config.LINE_TRACKER_L_PIN, config.LINE_TRACKER_R_PIN,
        config.TURN_TRACKER_L_PIN, config.TURN_TRACKER_R_PIN,
        white_is_high=config.LINE_WHITE_IS_HIGH)
    follower = LineFollower(drive, sensors,
                            speed=config.FOLLOW_SPEED,
                            steer=config.FOLLOW_STEER,
                            reverse_speed=config.RECOVER_SPEED,
                            lost_timeout_ms=config.LOST_TIMEOUT_MS)
    junctions = JunctionDetector(sensors,
                                 confirm_ms=config.JUNCTION_CONFIRM_MS,
                                 clear_ms=config.JUNCTION_CLEAR_MS)
    left, right = build_pair(config)
    bay = BayDetector(left, right,
                      full_mm=config.BAY_FULL_MM,
                      confirm_reads=config.BAY_CONFIRM_READS,
                      timeout_ms=config.BAY_TIMEOUT_MS)

    print("threshold: {} mm, {} far reads to call a bay empty"
          .format(config.BAY_FULL_MM, config.BAY_CONFIRM_READS))
    print("measuring the {} side. Press the button to start.".format(SIDE))
    button.wait_for_press()
    beacon.on()

    try:
        while True:
            if button.was_pressed():
                return
            beacon.update()

            if junctions.poll() != NO_JUNCTION:
                bay.start(SIDE)
                print("junction {} - measuring".format(bay.count + 1))

            result = bay.poll()
            if result != NONE:
                print("  bay {}: {}  ({} mm)".format(
                    bay.count, name(result), bay.distance()))

            follower.tick()
            sleep_ms(config.LOOP_PERIOD_MS)
    finally:
        drive.stop()
        beacon.off()
        print("stopped after {} bays".format(bay.count))


if __name__ == "__main__":
    test_bay()
