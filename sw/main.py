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

from nav.navigator import State
from utime import sleep_ms, ticks_ms, ticks_diff

import config
from drivers.button import Button
from drivers.drive import Drive
from drivers.indicators import FlashingLamp
from drivers.line_sensor import LineSensorArray
from nav.follower import LineFollower
from nav.junction import NONE, JunctionDetector, name
from nav.map import is_dead_end


def _perform_turn(button, beacon, drive, sensors, turn):
    """Rotate off the line the AGV is on and onto the next one.

    Three phases, with the durations measured on the table during M1:

      drive over  carry the wheels past the junction, so the AGV pivots
                  beyond the marking rather than on top of it
      commit      rotate without checking the trackers, so they clear the
                  line being left; without this the turn finishes instantly
                  on a line it has not left yet
      seek        keep rotating until the line is reacquired

    M1 ran these as fixed sleeps, which left the stop button unread for up
    to 4.4 seconds at a time. They are deadlines inside the loop now, so the
    motion is unchanged but the button and beacon are serviced every pass.

    Returns True once the line is found and False if the stop button was
    pressed. Raises if the line never appears.
    """
    if turn == "straight":
        return True

    if turn == "left":
        direction = -1
    elif turn == "right":
        direction = 1
    elif turn == "uturn":
        direction = None
    else:
        raise ValueError("Unknown route turn: {}".format(turn))

    if turn == "uturn":
        drive_over_ms = config.UTURN_DRIVE_OVER_MS
        commit_ms = config.UTURN_COMMIT_MS
    else:
        drive_over_ms = config.TURN_DRIVE_OVER_MS
        commit_ms = config.TURN_COMMIT_MS
    seek_from_ms = drive_over_ms + commit_ms

    started_ms = ticks_ms()
    has_left_line = False

    while True:
        if button.was_pressed():
            drive.stop()
            return False

        beacon.update()
        elapsed = ticks_diff(ticks_ms(), started_ms)

        if elapsed >= config.TURN_TIMEOUT_MS:
            drive.stop()
            raise RuntimeError(
                "Timed out waiting for the line after a {} turn".format(turn))

        line_l, line_r = sensors.line_trackers()
        if not line_l or not line_r:
            has_left_line = True

        if elapsed < drive_over_ms:
            drive.move(config.FOLLOW_SPEED)
        else:
            # Only accept the new line once the commit phase is over, so the
            # line being left cannot be mistaken for the one being sought.
            if elapsed >= seek_from_ms and has_left_line and line_l and line_r:
                drive.stop()
                return True

            if turn == "uturn":
                drive.u_turn(config.FOLLOW_SPEED)
            else:
                drive.rotate(direction, config.TURN_SPEED)

        sleep_ms(config.LOOP_PERIOD_MS)


def _reverse_out_of_bay(button, beacon, drive):
    """Back out of a dead-end bay after the line has run out.

    A bay is a spur with nothing past it, so there is no line to follow back
    out; the AGV reverses for a measured time until it is far enough along
    the spur to turn. Measured on the table during M1.

    Returns False if the stop button was pressed.
    """
    started_ms = ticks_ms()
    while ticks_diff(ticks_ms(), started_ms) < config.BAY_REVERSE_MS:
        if button.was_pressed():
            drive.stop()
            return False
        beacon.update()
        drive.move(-config.FOLLOW_SPEED)
        sleep_ms(config.LOOP_PERIOD_MS)

    drive.stop()
    return True


def run(button, beacon, drive, sensors, follower, junction_detector, state):
    """The flowchart loop.  Returns when the stop button is pressed."""
    beacon.on()                         # amber beacon on - the AGV is live

    while True:
        # --- stop button pressed? --------------------------------------
        if button.was_pressed():
            return                      # --> End

        # --- flash LED -------------------------------------------------
        beacon.update()                 # toggles only when 250 ms has passed

        # --- end of a bay? ---------------------------------------------
        # The line simply stops at a bay, so losing it there is arrival
        # rather than a fault. M1 recognised this by naming one bay node;
        # every bay is a dead end on the map, so the rule needs no names.
        line_l, line_r = sensors.line_trackers()
        if not line_l and not line_r and is_dead_end(state.current_node):
            print("Bay reached at {} - backing out".format(state.current_node))
            if not _reverse_out_of_bay(button, beacon, drive):
                return
            turn = state.junction_turn()
            if turn is None:
                print("Route complete at {}".format(state.current_node))
                return
            print("Leaving the bay: {}".format(turn))
            if not _perform_turn(button, beacon, drive, sensors, turn):
                return

        # --- turn? -----------------------------------------------------
        signature = junction_detector.poll()
        if signature != NONE:
            turn = state.junction_turn()
            if turn is not None:
                print("Junction detected: {} - route instruction: {}".format(
                    name(signature), turn))
                if not _perform_turn(button, beacon, drive, sensors, turn):
                    return
            else:
                print("Junction detected: {} - route complete".format(
                    name(signature)))
                return

        # --- follow line -----------------------------------------------
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
    junction_detector = JunctionDetector(
        sensors,
        confirm_ms=config.JUNCTION_CONFIRM_MS,
        clear_ms=config.JUNCTION_CLEAR_MS)
    state = State(initial_node=config.INITIAL_NODE,
                  initial_direction=config.INITIAL_DIRECTION,
                  sequence=config.SEQUENCE)

    print("AGV ready - press the button to start")
    button.wait_for_press()

    try:
        run(button, beacon, drive, sensors, follower, junction_detector, state)
    finally:
        # Runs on a normal stop, on Ctrl-C from Thonny, and on any unhandled
        # exception raised inside the loop.
        drive.stop()
        beacon.off()
        print("AGV stopped")


if __name__ == "__main__":
    main()
