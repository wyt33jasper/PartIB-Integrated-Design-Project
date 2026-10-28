"""AGV main program - IDP M1 2026.

Implements the team flowchart:

    Main --> [stop button pressed?] --yes--> End
                     | no
                     v
              Flash amber LED
                     v
         [shelf bay beside us?] --yes--> back up to it and measure; turn
                     | no                in and deliver, or move on
                     v
        [delivery bay or junction?] --yes--> deliver / collect / turn
                     | no
                     v
               Line follower
                     |
                     +--------> back to the stop check

Where to go is decided by nav/mission.py (collect, deliver or go home);
this file drives the hardware and reports back what happened.

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
from drivers.distance import build_rear
from drivers.drive import Drive
from drivers.indicators import FlashingLamp, PoleIndicator
from drivers.line_sensor import LineSensorArray
from drivers.magnet import MagnetSensor
from nav.bay import NONE as BAY_NONE, BayDetector, name as bay_name
from nav.bay import EMPTY as BAY_EMPTY
from nav.follower import LineFollower
from nav.junction import NONE, JunctionDetector, name
from nav.mission import COLLECT_BAYS, Mission
from drivers.servo import Servo

DELIVERY_BAYS = ("RB1_BAY", "RB2_BAY", "RB3_BAY", "RB4_BAY", "RB5_BAY", "RB6_BAY",
                 "LB1_BAY", "LB2_BAY", "LB3_BAY", "LB4_BAY", "LB5_BAY", "LB6_BAY",
                 "RU1_BAY", "RU2_BAY", "RU3_BAY", "RU4_BAY", "RU5_BAY", "RU6_BAY",
                 "LU1_BAY", "LU2_BAY", "LU3_BAY", "LU4_BAY", "LU5_BAY", "LU6_BAY",)

#these are bays turn-out that will require state advancement (note during RB1_BAY turn-out, AGV wont skip a junction, thus ...1_BAY are excluded)
MISS_CASE_DELIVERY_BAYS = tuple(bay for bay in DELIVERY_BAYS if not bay.endswith("1_BAY"))

def _just_collected(state):
    """True at the junction reached straight after leaving a collect bay.

    Must be called before junction_turn(), while current_node is still the
    junction the AGV is physically at.
    """
    return state.previous_node() in COLLECT_BAYS

def _at_collect_bay(state):
    return state.current_node in COLLECT_BAYS

def _at_delivery_bay(sensors, state):
    line_l, line_r = sensors.line_trackers()
    return not line_l and not line_r and state.current_node in DELIVERY_BAYS

def _wait_ms(button, beacon, ms, until=None):
    """Wait ms while still polling the stop button and flashing the beacon.

    The motors keep whatever command they were last given.  Returns False if
    the stop button was pressed during the wait.  If until is given, the wait
    ends early as soon as until() returns True.
    """
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < ms:
        if button.was_pressed():
            return False
        beacon.update()
        if until is not None and until():
            return True
        sleep_ms(config.LOOP_PERIOD_MS)
    return True

def _read_pole(button, beacon, drive, magnet, poles):
    """Stop, read the gripped box's magnet pole and show it on the pole lamps.

    The lamp stays lit while the box is carried. Returns False if the stop
    button was pressed.
    """
    drive.stop()
    magnet.reset()
    if not _wait_ms(button, beacon, config.MAGNET_READ_TIMEOUT_MS,
                    until=lambda: magnet.poll() is not None):
        return False
    poles.show(magnet.pole)
    if magnet.pole is None:
        print("No pole read - raw {}, idle {}".format(magnet.raw, magnet.centre))
    else:
        print("Package pole: {}".format(magnet.pole))
    return True

def _drive_ms(button, beacon, drive, speed, ms):
    """Drive at speed for ms, then stop.  False if the stop button was hit."""
    drive.move(speed)
    moved = _wait_ms(button, beacon, ms)
    drive.stop()
    return moved


def _rotate_ms(button, beacon, drive, direction, ms):
    """Rotate in place for ms, then stop.

    Used where _perform_turn() cannot be: pointing out of the corridor there
    is no line for the trackers to end the rotation on.
    """
    drive.rotate(direction, config.TURN_SPEED)
    turned = _wait_ms(button, beacon, ms)
    drive.stop()
    return turned


def _measure(button, beacon, detector, ms):
    """Poll the bay detector for up to ms.

    Returns EMPTY or FULL once decided, BAY_NONE if ms ran out with no
    verdict, or None if the stop button was pressed.  The motors keep
    whatever command they were last given.
    """
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < ms:
        if button.was_pressed():
            return None
        beacon.update()
        verdict = detector.poll()
        if verdict != BAY_NONE:
            return verdict
        sleep_ms(config.LOOP_PERIOD_MS)
    return BAY_NONE


def _acquire_line(button, beacon, drive, sensors, ms):
    """Creep forward until both inner trackers are back on the line.

    The bay spin is open loop, so the AGV can finish it just off the spur
    line.  Handing back to the main loop there would read exactly like the
    far end of a delivery bay, where the line also runs out - and the box
    would be released in the corridor.
    """
    start = ticks_ms()
    drive.move(config.BAY_REVERSE_SPEED)
    while ticks_diff(ticks_ms(), start) < ms:
        if button.was_pressed():
            drive.stop()
            return False
        beacon.update()
        line_l, line_r = sensors.line_trackers()
        if line_l and line_r:
            drive.stop()
            return True
        sleep_ms(config.LOOP_PERIOD_MS)
    drive.stop()
    print("WARNING: no line after the bay spin - CHECK BAY_SPIN_MS")
    return False


# Which way to rotate to put the back towards the bay: a bay on the left is
# behind us once we have turned right.  drive.rotate() takes -1 left, +1 right.
_BACK_TOWARDS = {"left": 1, "right": -1}


def _probe_bay(button, beacon, drive, sensors, detector, bay, side):
    """Back up to a shelf bay, measure it, and face the answer.

    The sensor looks straight back, so the AGV rotates away from the bay,
    reverses up to its mouth and measures there.  A near reading during the
    reverse ends it early - that is a full bay, and stopping there saves
    nosing into the package.

    Returns EMPTY with the AGV facing into the bay ready to drive in, FULL
    with it back on the corridor heading, or None on a stop press.
    """
    away = _BACK_TOWARDS[side]

    if not _rotate_ms(button, beacon, drive, away, config.BAY_FACE_AWAY_MS):
        return None

    detector.start(bay)
    drive.move(-config.BAY_REVERSE_SPEED)
    since_ms = ticks_ms()
    verdict = _measure(button, beacon, detector, config.BAY_APPROACH_MS)
    reversed_ms = ticks_diff(ticks_ms(), since_ms)   # how far back to come out
    drive.stop()
    if verdict is None:
        detector.cancel()
        return None

    if verdict == BAY_NONE:
        # At the mouth with nothing close.  Stand still and let a run of far
        # readings call it empty - on the way in they could be the corridor.
        detector.settle()
        verdict = _measure(button, beacon, detector, config.BAY_TIMEOUT_MS)
        if verdict is None:
            detector.cancel()
            return None
        if verdict == BAY_NONE:
            verdict = detector.give_up()        # no clear reading - call it full
    print("{} is {} ({} mm)".format(bay, bay_name(verdict), detector.last_mm))

    # Come back out, then aim into the bay or rejoin the corridor heading.
    if not _drive_ms(button, beacon, drive, config.BAY_REVERSE_SPEED,
                     reversed_ms):
        return None
    if verdict == BAY_EMPTY:
        if not _rotate_ms(button, beacon, drive, away, config.BAY_SPIN_MS):
            return None
        # Settle onto the spur line before the main loop can mistake being
        # off it for having reached the end of the bay.
        if not _acquire_line(button, beacon, drive, sensors,
                             config.BAY_ACQUIRE_MS):
            return None
    elif not _rotate_ms(button, beacon, drive, -away,
                        config.BAY_FACE_AWAY_MS):
        return None                     # the follower re-finds the corridor
    return verdict


def _perform_turn(button, beacon, drive, sensors, turn, short_drive_over=False,
                  driven_ms=0):
    """Rotate until the inner sensors leave the old line and reacquire one.

    Each turn ends on the first line reacquisition. U-turns use the drive's
    dedicated U-turn motion; junction polling and line following are paused.
    short_drive_over uses the shorter drive-forward time before rotating.
    driven_ms is drive-over already done while measuring bays at the junction.
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
                u_turn_counter += 1

                # drive forward first so the spin lands on the line; reused after delivery reversing too
                drive.move(config.FOLLOW_SPEED)
                drive_over_ms = max(0, config.UTURN_DRIVE_OVER_MS - driven_ms)  # what is left after measuring
                if not _wait_ms(button, beacon, drive_over_ms):
                    drive.stop()
                    return False

                drive.u_turn(config.FOLLOW_SPEED)
                if not _wait_ms(button, beacon, config.UTURN_SPIN_MS):
                    drive.stop()
                    return False

            drive.u_turn(config.FOLLOW_SPEED)

        else:
            if drive_over_counter ==0:
                drive_over_counter += 1

                drive.move(config.FOLLOW_SPEED)
                if short_drive_over:
                    drive_over_ms = config.TURN_DRIVE_OVER_SHORT_MS
                else:
                    drive_over_ms = config.TURN_DRIVE_OVER_MS
                drive_over_ms = max(0, drive_over_ms - driven_ms)   # what is left after measuring
                if not _wait_ms(button, beacon, drive_over_ms):
                    drive.stop()
                    return False
                
            if turn_time_counter == 0:
                turn_time_counter += 1

                drive.rotate(direction, config.TURN_SPEED)
                if not _wait_ms(button, beacon, 400):
                    drive.stop()
                    return False
                
            drive.rotate(direction, config.TURN_SPEED)
        sleep_ms(config.LOOP_PERIOD_MS)


def _turn_at_junction(button, beacon, drive, sensors, state, label,
                      short_drive_over, misses_next, driven_ms=0):
    """Take the next planned turn. Returns False if the run should end.

    The run ends on a stop press, or when the route is complete (back home).
    """
    turn = state.junction_turn()        # advances the state to the next node
    if turn is None:
        print("{} - route complete".format(label))
        return False
    print("{} - route instruction: {}".format(label, turn))
    if not _perform_turn(button, beacon, drive, sensors, turn, short_drive_over,
                         driven_ms):
        return False
    if misses_next:
        state.junction_turn()
    return True


def run(button, beacon, drive, sensors, follower, junction_detector, state, servo,
        magnet, poles, detector, mission):
    """The flowchart loop.  Returns when the stop button is pressed or the run ends."""
    beacon.on()                         # amber beacon on - the AGV is live
    mission.start()                     # plans the route to the first pickup
    print(mission.report())

    while True:
        # --- stop button pressed? --------------------------------------
        if button.was_pressed():
            return                      # --> End
        # --- flash LED -------------------------------------------------
        beacon.update()                 # toggles only when 250 ms has passed

        # --- turn? -------------------------------------------------

        signature = junction_detector.poll()

        #delivery bay cannot be normally recognised as a junction (no signature), so check it separately
        if _at_delivery_bay(sensors, state):
            #release box
            servo.set_Grabber_angle(config.GRABBER_OPEN)
            poles.off()

            #reverse and set lifter bck to ready-to-pickup state 0deg
            drive.move(-config.FOLLOW_SPEED)
            if not _wait_ms(button, beacon, 1500):
                return
            servo.set_Lifter_angle(config.LIFTER_DOWN)

            mission.delivered()             # plans the next pickup, or home
            print(mission.report())

            #perform turn
            turn = state.junction_turn()
            print("Delivery bay reached - route instruction: {}".format(turn))
            if not _perform_turn(button, beacon, drive, sensors, turn):
                return

        elif signature != NONE:   # at every junction (readable, unlike at delivery bays)

            if _at_collect_bay(state):
                #collect/grab box then lift off ground
                servo.set_Grabber_angle(config.GRABBER_CLOSED)
                if not _wait_ms(button, beacon, 200):
                    return
                servo.set_Lifter_angle(config.LIFTER_CARRY)

                if not _read_pole(button, beacon, drive, magnet, poles):
                    return
                if magnet.pole is None:
                    # nothing gripped after all - let go and carry on collecting
                    servo.set_Grabber_angle(config.GRABBER_OPEN)
                    servo.set_Lifter_angle(config.LIFTER_DOWN)
                mission.collected(magnet.pole)  # plans the delivery, or the next pickup
                print(mission.report())

            short_drive_over = _just_collected(state)   # before junction_turn() advances the state
            misses_next = state.previous_node() in MISS_CASE_DELIVERY_BAYS   # determines whether the next junction will be missed
            label = "Junction detected: {}".format(name(signature))   # for the console

            # --- shelf bay beside us? ----------------------------------
            side, bay = mission.bay_here()   # before junction_turn() advances the state
            if side is not None:
                drive.stop()
                # Raise now: the lifter has the whole manoeuvre to get there.
                servo.set_Lifter_angle(config.LIFTER_SHELF)
                if _probe_bay(button, beacon, drive, sensors, detector,
                              bay, side) is None:
                    return
                junction_detector.pause()   # polling was off for several seconds
                if detector.bay_is_empty():
                    mission.take_bay(bay)
                    state.junction_turn()   # the spin has already aimed us in
                    print("Delivering into {} - {}".format(bay, mission.report()))
                    continue
                mission.skip_bay()          # full: on down the shelf, or give up
                if not _turn_at_junction(button, beacon, drive, sensors, state,
                                         label, False, misses_next):
                    return
                continue

            #perform turn
            if not _turn_at_junction(button, beacon, drive, sensors, state, label,
                                     short_drive_over, misses_next):
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
                  sequence = [config.INITIAL_NODE])    # mission.start() plans the route
    servo = Servo(ServoLifter_pin = config.SERVO_LIFTER_PIN,
                  ServoGrabber_pin = config.SERVO_GRABBER_PIN)
    magnet = MagnetSensor(config.MAGNET_ADC_PIN,
                          deadband=config.MAGNET_DEADBAND,
                          north_is_high=config.MAGNET_NORTH_IS_HIGH,
                          poll_ms=config.MAGNET_POLL_MS,
                          confirm_reads=config.MAGNET_CONFIRM_READS)
    poles = PoleIndicator(config.RED_LED_PIN, config.GREEN_LED_PIN)
    detector = BayDetector(build_rear(config),
                           full_mm=config.BAY_FULL_MM,
                           near_reads=config.BAY_NEAR_READS,
                           confirm_reads=config.BAY_CONFIRM_READS)
    mission = Mission(state,
                      first_pickup=config.FIRST_PICKUP,
                      home=config.HOME_NODE,
                      run_limit_ms=config.RUN_LIMIT_MS,
                      cycle_reserve_ms=config.CYCLE_RESERVE_MS,
                      empty_bays_before_home=config.EMPTY_BAYS_BEFORE_HOME)

    # Calibrate in the start box, gripper empty, before anything moves.
    print("Magnet idle = {}".format(
        magnet.calibrate(config.MAGNET_CALIBRATE_SAMPLES)))

    print("AGV ready - press the button to start")
    button.wait_for_press()

    #imitalising grabber and lifter
    servo.set_Grabber_angle(config.GRABBER_OPEN)
    servo.set_Lifter_angle(config.LIFTER_DOWN)

    try:
        run(button, beacon, drive, sensors, follower, junction_detector, state, servo,
            magnet, poles, detector, mission)
    finally:
        # Runs on a normal stop, on Ctrl-C from Thonny, and on any unhandled
        # exception raised inside the loop.
        drive.stop()
        beacon.off()
        poles.off()
        print("AGV stopped")


if __name__ == "__main__":
    main()
