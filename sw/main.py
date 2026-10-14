"""AGV main program - IDP M1 2026.

Implements the team flowchart:

    Main --> [stop button pressed?] --yes--> End
                     | no
                     v
              Flash amber LED
                     v
               Line follower
                     |
                     +--------> back to the stop check

Two rules hold the design together:

  * Nothing inside the loop blocks.  Each call returns immediately, so the
    stop button is tested every few milliseconds rather than whenever a
    sleep() happens to finish.
  * The motors are stopped on every exit path, including an unhandled
    exception, so a software fault cannot leave the AGV driving.

main.py runs automatically when the Pico is powered up, so the AGV waits for
a deliberate start press before it moves.
"""

from utime import sleep_ms

import config
from drivers.button import Button
from drivers.drive import Drive
from drivers.indicators import FlashingLamp
from drivers.line_sensor import LineSensorArray
from nav.follower import LineFollower


def run(button, beacon, drive, follower):
    """The flowchart loop.  Returns when the stop button is pressed."""
    beacon.on()                         # amber beacon on - the AGV is live

    while True:
        # --- stop button pressed? --------------------------------------
        if button.was_pressed():
            return                      # --> End

        # --- flash LED -------------------------------------------------
        beacon.update()                 # toggles only when 250 ms has passed

        # --- line follower ---------------------------------------------
        follower.tick()                 # one control step, then straight back

        sleep_ms(config.LOOP_PERIOD_MS)


def main():
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
    follower = LineFollower(drive, sensors,
                            speed=config.FOLLOW_SPEED,
                            steer=config.FOLLOW_STEER,
                            reverse_speed=config.RECOVER_SPEED,
                            lost_timeout_ms=config.LOST_TIMEOUT_MS)

    print("AGV ready - press the button to start")
    button.wait_for_press()

    try:
        run(button, beacon, drive, follower)
    finally:
        # Runs on a normal stop, on Ctrl-C from Thonny, and on any unhandled
        # exception raised inside the loop.
        drive.stop()
        beacon.off()
        print("AGV stopped")


if __name__ == "__main__":
    main()
