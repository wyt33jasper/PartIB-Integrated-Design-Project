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
from drivers.servo import Servo
from drivers.distance import build_pair
from nav.bay import BayDetector
from nav.bay import EMPTY as BAY_EMPTY


def _perform_turn(button, beacon, drive, sensors, turn, state):
    """Rotate until the inner sensors leave the old line and reacquire one.

    Each turn ends on the first line reacquisition. U-turns use the drive's
    dedicated U-turn motion; junction polling and line following are paused.
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

    deadline = ticks_ms()
    has_left_line = False
    drive_over_counter = 0
    turn_time_counter =0
    u_turn_counter = 0
    while True:
        if button.was_pressed():
            drive.stop()
            return False

        beacon.update()
        line_l, line_r = sensors.line_trackers()

        if not line_l or not line_r: #if either line sensor doesn’t see the line, mark that the robot has left it. note elif is not ran until "not line_l or not line_r" become false, that is when robot is back in line
            has_left_line = True
        elif has_left_line:
            drive.stop()
            return True

        if ticks_diff(ticks_ms(), deadline) >= config.TURN_TIMEOUT_MS:
            drive.stop()
            raise RuntimeError(
                "Timed out waiting for the line after a {} turn".format(turn))

        if turn == "uturn":
            if u_turn_counter == 0:
                drive.move(config.FOLLOW_SPEED)
                sleep_ms(700)
                drive.u_turn(config.FOLLOW_SPEED)
                sleep_ms(1200)
                u_turn_counter +=1
            drive.u_turn(config.FOLLOW_SPEED)
        else:
            if drive_over_counter ==0:
                drive.move(config.FOLLOW_SPEED)
                if state.next_node() == "RB1" and state.current_node == "R2":
                    sleep_ms(350)
                else:
                    sleep_ms(550)
                drive_over_counter += 1
            if turn_time_counter == 0:
                drive.rotate(direction, config.TURN_SPEED)
                sleep_ms(400)
                turn_time_counter += 1
            drive.rotate(direction, config.TURN_SPEED)
        sleep_ms(config.LOOP_PERIOD_MS)


def run(button, beacon, drive, sensors, follower, junction_detector, state,
        servo, bay, bay_side=None):
    """The flowchart loop.  Returns when the stop button is pressed.

    bay_side arms the empty-bay scan: set it to "left" or "right" when the
    AGV enters a rack corridor, and leave it None otherwise. While it is
    None the scan block below does nothing and this loop behaves exactly
    as it did before the scan was added.
    """
    beacon.on()                         # amber beacon on - the AGV is live

    while True:
        # --- stop button pressed? --------------------------------------
        if button.was_pressed():
            return                      # --> End
        # --- flash LED -------------------------------------------------
        beacon.update()                 # toggles only when 250 ms has passed
        # --- turn? -------------------------------------------------
        
        signature = junction_detector.poll()

        # --- empty bay scan ---------------------------------------------
        # The distance sensors sit level with the turn trackers, so a
        # junction marking means a bay is beside us right now. That is the
        # only moment we measure - the rack's own dividers would otherwise
        # read as full bays. The robot keeps driving throughout.
        if bay_side is not None:
            if signature != NONE:
                bay.start(bay_side)
            if bay.poll() == BAY_EMPTY:
                print("empty bay on the", bay_side, "-", bay.distance(), "mm")
                turn = bay_side             # "left"/"right" already
                bay_side = None             # stop scanning, we are turning in
                if not _perform_turn(button, beacon, drive, sensors, turn,
                                     state):
                    return
                continue

        line_l, line_r = sensors.line_trackers()
        
        if not line_l and not line_r and state.current_node =="RB2_BAY":
            signature = "AT RB2_BAY"
        print(state.current_node, state.current_direction)
        if signature == "AT RB2_BAY":
            servo.set_Grabber_angle(50)
            #reverse
            drive.move(-config.FOLLOW_SPEED)
            sleep_ms(1500)
            servo.set_Lifter_angle(0)
            #perform turn
            turn = state.junction_turn()
            print("turn:", turn)
            print("Junction detected: {} - route instruction: {}".format(
                    name(signature), turn))
            if not _perform_turn(button, beacon, drive, sensors, turn, state): 
                return #to run()
        elif signature != NONE:
            bay.cancel()        # drop any half-finished bay measurement
            if state.current_node == "R1_BAY":
                servo.set_Grabber_angle(10)
                sleep_ms(200)
                servo.set_Lifter_angle(20)
            if state.current_node == "RB2":
                servo.set_Lifter_angle(45)
            turn = state.junction_turn()
            print("turn:", turn)
            if turn is not None:
                print("Junction detected: {} - route instruction: {}".format(
                    name(signature), turn))
                if state.current_node == "RB1" and state.next_node() == "R2":
                    print(state.junction_turn())
                if not _perform_turn(button, beacon, drive, sensors, turn, state):
                    return #to run()
            else:
                print("Junction detected: {} - route complete".format(
                    name(signature)))
                return

        # --- follow line -------------------------------------------------
        follower.tick()                 # line follower, one control step, then straight back

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
    state = State(initial_node = config.INITIAL_NODE, 
                  initial_direction = config.INITIAL_DIRECTION, 
                  sequence = config.SEQUENCE)
    servo = Servo(ServoLifter_pin = 13, ServoGrabber_pin = 15) # grabber on servo 1, lifter on servo 2
    left, right = build_pair(config)
    bay = BayDetector(left, right,
                      full_mm=config.BAY_FULL_MM,
                      confirm_reads=config.BAY_CONFIRM_READS,
                      timeout_ms=config.BAY_TIMEOUT_MS)

    print("AGV ready - press the button to start")
    button.wait_for_press()
    
    #imitalising grabber and lifter
    servo.set_Grabber_angle(70)
    servo.set_Lifter_angle(0)
    
    try:
        run(button, beacon, drive, sensors, follower, junction_detector,
            state, servo, bay)
    finally:
        # Runs on a normal stop, on Ctrl-C from Thonny, and on any unhandled
        # exception raised inside the loop.
        drive.stop()
        beacon.off()
        print("AGV stopped")


if __name__ == "__main__":
    main()
