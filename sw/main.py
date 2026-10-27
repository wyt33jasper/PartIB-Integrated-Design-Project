"""AGV main program - IDP M1 2026.

The run, as a loop:

    Main --> [stop button pressed?] --yes--> End
                     | no
                     v
              Flash amber LED
                     v
         [line run out in a bay?] --yes--> grip or release, back out, turn
                     | no
                     v
            [junction detected?] --yes--> ask the mission for the turn
                     | no
                     v
               Line follower
                     |
                     +--------> back to the stop check

Where M1 drove a route typed out in config.py, the turns now come from
nav/mission.py, which plans each leg from a goal. The loop's job is to run
the hardware and report two events - a junction, and the line running out in
a bay - and to carry out whatever the mission asks for in reply. It makes no
route decisions of its own.

Two rules hold the design together:

  * Nothing inside the loop blocks.  Each call returns immediately, so the
    stop button is tested every few milliseconds rather than whenever a
    sleep() happens to finish.  The gripper movements and the turn phases
    are deadlines checked inside the loop for exactly this reason: a grip
    takes a couple of seconds, and sleeping through it would leave the AGV
    holding a package with its safety control unread.
  * The motors are stopped on every exit path, including an unhandled
    exception, so a software fault cannot leave the AGV driving.

main.py runs automatically when the Pico is powered up, so the AGV waits for
a deliberate start press before it moves.
"""

from utime import sleep_ms, ticks_ms, ticks_diff

import config
from drivers.button import Button
from drivers.drive import Drive
from drivers.gripper import gripper_from_config
from drivers.indicators import FlashingLamp, PoleIndicator
from drivers.line_sensor import LineSensorArray
from drivers.magnet import MagnetSensor, probe_from_config
from drivers.rangefinder import scanner_from_config
from nav.follower import LineFollower
from nav.junction import NONE, JunctionDetector, name
from nav.mission import Mission, PICK_UP, PUT_DOWN
from nav.navigator import State


class Robot:
    """Everything the loop touches, so the helpers take one argument.

    service() is the part that matters: whatever the AGV is doing - turning,
    gripping, reversing out of a bay - these all have to keep running, so
    every wait in this file is a loop around service() rather than a sleep.
    """

    def __init__(self, button, beacon, drive, sensors, follower, junctions,
                 poles, magnet, scanner, gripper, mission):
        self.button = button
        self.beacon = beacon
        self.drive = drive
        self.sensors = sensors
        self.follower = follower
        self.junctions = junctions
        self.poles = poles
        self.magnet = magnet
        self.scanner = scanner
        self.gripper = gripper
        self.mission = mission

    def service(self):
        """One pass of everything that has to keep running. Never blocks."""
        self.beacon.update()
        self.gripper.update()
        self.scanner.poll()
        self.magnet.poll()
        self.mission.tick()

        # The lamps show the package being carried, which stays lit for the
        # whole delivery, and fall back to the live reading while a package
        # is being picked up - so the judges see the identification happen.
        # Both go dark once it is put down, which keeps the display honest.
        self.poles.show(self.mission.carrying or self.magnet.pole)


def _wait(robot, until, timeout_ms=0):
    """Service the robot until until() is true. False if stopped or timed out.

    Returns True when until() became true, False if the stop button was
    pressed, and None on a timeout.
    """
    started_ms = ticks_ms()
    while True:
        if robot.button.was_pressed():
            robot.drive.stop()
            return False
        robot.service()
        if until():
            return True
        if timeout_ms and ticks_diff(ticks_ms(), started_ms) >= timeout_ms:
            return None
        sleep_ms(config.LOOP_PERIOD_MS)


def _perform_turn(robot, turn):
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

    # The outer trackers sweep across lines throughout a turn. Suspending
    # detection means none of that is counted as a junction, and requires a
    # clear window before the next one is, so the junction just turned at
    # cannot be counted a second time on the way out of it.
    robot.junctions.pause()

    started_ms = ticks_ms()
    has_left_line = False

    while True:
        if robot.button.was_pressed():
            robot.drive.stop()
            return False

        robot.service()
        elapsed = ticks_diff(ticks_ms(), started_ms)

        if elapsed >= config.TURN_TIMEOUT_MS:
            robot.drive.stop()
            raise RuntimeError(
                "Timed out waiting for the line after a {} turn".format(turn))

        line_l, line_r = robot.sensors.line_trackers()
        if not line_l or not line_r:
            has_left_line = True

        if elapsed < drive_over_ms:
            robot.drive.move(config.FOLLOW_SPEED)
        else:
            # Only accept the new line once the commit phase is over, so the
            # line being left cannot be mistaken for the one being sought.
            if elapsed >= seek_from_ms and has_left_line and line_l and line_r:
                robot.drive.stop()
                robot.junctions.pause()
                return True

            if turn == "uturn":
                robot.drive.u_turn(config.FOLLOW_SPEED)
            else:
                robot.drive.rotate(direction, config.TURN_SPEED)

        sleep_ms(config.LOOP_PERIOD_MS)


def _reverse_out_of_bay(robot):
    """Back out of a dead-end bay after the line has run out.

    A bay is a spur with nothing past it, so there is no line to follow back
    out; the AGV reverses for a measured time until it is far enough along
    the spur to turn. Measured on the table during M1.

    Returns False if the stop button was pressed.
    """
    started_ms = ticks_ms()

    def done():
        robot.drive.move(-config.FOLLOW_SPEED)
        return ticks_diff(ticks_ms(), started_ms) >= config.BAY_REVERSE_MS

    result = _wait(robot, done)
    robot.drive.stop()
    return result is True


def _collect(robot):
    """Close on the package and read its pole.

    Returns (keep_going, pole). A pole of None means there was nothing to
    pick up: either the loading bay is empty or the magnet never confirmed,
    and both have the same consequence for the plan.

    The Hall pair does double duty here. Neither sensor reads high until the
    magnet is in range, so "a pole is readable" and "the package is close
    enough to grip" are the same signal - which is why nothing else has to
    measure the approach.
    """
    robot.magnet.reset()
    robot.gripper.close()

    result = _wait(robot,
                   lambda: robot.magnet.pole is not None
                   and not robot.gripper.busy,
                   timeout_ms=config.COLLECT_TIMEOUT_MS)
    if result is False:
        return False, None
    if result is None:
        print("  no package identified here after {} ms".format(
            config.COLLECT_TIMEOUT_MS))
    return True, robot.magnet.pole


def _release(robot):
    """Open the gripper and wait for the movement to finish."""
    robot.gripper.open()
    return _wait(robot, lambda: not robot.gripper.busy) is True


def _handle_bay(robot):
    """Do whatever the bay the AGV has driven into is for.

    Returns False if the run should end.
    """
    mission = robot.mission
    job = mission.at_bay()

    if job == PICK_UP:
        print("  collecting at {}".format(mission.target))
        keep_going, pole = _collect(robot)
        if not keep_going:
            return False
        if pole is None:
            # Nothing gripped, so let go rather than carrying the mechanism
            # around closed on thin air.
            if not _release(robot):
                return False
        else:
            print("  identified {} pole".format(pole))
        mission.collected(pole)
        return True

    if job == PUT_DOWN:
        print("  delivering at {}".format(mission.target))
        if not _release(robot):
            return False
        mission.dropped()
        return True

    print("  unexpected dead end at {} in phase {}".format(
        robot.mission.state.current_node, mission.phase))
    return False


def run(robot):
    """The flowchart loop.  Returns when the run ends."""
    robot.beacon.on()                   # amber beacon on - the AGV is live

    # The first instruction is a turn where the AGV stands: it may be parked
    # facing away from its first destination, and there is no junction to
    # wait for before setting off.
    turn = robot.mission.start()
    print(robot.mission.report())
    if turn is not None and not _perform_turn(robot, turn):
        return

    # Leaving the start box drags the trackers over whatever marking the
    # start node has. Requiring a clear window first means that is not
    # counted as the first junction.
    robot.junctions.pause()

    while True:
        # --- stop button pressed? --------------------------------------
        if robot.button.was_pressed():
            return                      # --> End

        # --- beacon, gripper, sensors, lamps, clock --------------------
        robot.service()

        # --- line run out in a bay? ------------------------------------
        # The line simply stops at a bay, so losing it there is arrival
        # rather than a fault - but only once the AGV has turned into the
        # spur, which is what expecting_bay checks.
        line_l, line_r = robot.sensors.line_trackers()
        if not line_l and not line_r and robot.mission.expecting_bay:
            robot.drive.stop()
            if not _handle_bay(robot):
                return
            if not _reverse_out_of_bay(robot):
                return
            turn = robot.mission.leaving_bay()
            print(robot.mission.report())
            if turn is None:
                return
            if not _perform_turn(robot, turn):
                return
            continue

        # --- an empty bay beside us? -----------------------------------
        # The side rangefinders look out of the chassis the whole time, so
        # this is asked every pass rather than at any particular place. The
        # AGV drives past the full bays without stopping and turns into the
        # first free one the moment the sensor reports it.
        turn = robot.mission.bay_beside()
        if turn is not None:
            print("free bay beside us at {} -> {}".format(
                robot.mission.state.current_node, turn))
            if not _perform_turn(robot, turn):
                return
            continue

        # --- junction? -------------------------------------------------
        signature = robot.junctions.poll()
        if signature != NONE:
            turn = robot.mission.at_junction()
            if turn is None:
                robot.drive.stop()
                print("Run complete: {}".format(robot.mission.report()))
                return
            print("{} at {} -> {}".format(name(signature),
                                          robot.mission.state.current_node,
                                          turn))
            if not _perform_turn(robot, turn):
                return
            continue

        # --- follow line -----------------------------------------------
        robot.follower.tick()           # one control step, then straight back

        sleep_ms(config.LOOP_PERIOD_MS)


def build():
    """Construct the whole robot from config.py."""
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
    junctions = JunctionDetector(
        sensors,
        confirm_ms=config.JUNCTION_CONFIRM_MS,
        clear_ms=config.JUNCTION_CLEAR_MS)
    poles = PoleIndicator(config.RED_LED_PIN, config.GREEN_LED_PIN)
    magnet = MagnetSensor(probe_from_config(config),
                          poll_ms=config.MAGNET_POLL_MS,
                          confirm_reads=config.MAGNET_CONFIRM_READS)
    scanner = scanner_from_config(config)
    gripper = gripper_from_config(config)

    state = State(initial_node=config.INITIAL_NODE,
                  initial_direction=config.INITIAL_DIRECTION)
    mission = Mission(state, scanner,
                      pickups=config.PICKUP_ORDER,
                      home=config.HOME_NODE,
                      run_limit_ms=config.RUN_LIMIT_MS,
                      cycle_reserve_ms=config.CYCLE_RESERVE_MS,
                      unknown_is_occupied=config.BAY_UNKNOWN_IS_OCCUPIED)

    print("bay scan: {} samples every {} ms, so a verdict looks back {} ms"
          .format(config.RANGER_WINDOW_SAMPLES, scanner.poll_ms,
                  scanner.window_ms))
    if not scanner.fitted:
        print("WARNING: no bay rangefinders (RANGER_MODE = {!r}). Every bay "
              "reads as unknown, so the scan will {}.".format(
                  config.RANGER_MODE,
                  "skip them all and carry every package home"
                  if config.BAY_UNKNOWN_IS_OCCUPIED
                  else "deliver to the first bay on each shelf"))
    if not getattr(gripper, "fitted", True):
        print("WARNING: no gripper fitted. Movements complete instantly, so "
              "the navigation runs but nothing is actually carried.")

    return Robot(button, beacon, drive, sensors, follower, junctions,
                 poles, magnet, scanner, gripper, mission)


def main():
    robot = build()

    print("AGV ready - press the button to start")
    robot.button.wait_for_press()

    try:
        run(robot)
    finally:
        # Runs on a normal stop, on Ctrl-C from Thonny, and on any unhandled
        # exception raised inside the loop.
        robot.drive.stop()
        robot.gripper.stop()
        robot.beacon.off()
        robot.poles.off()
        print("AGV stopped")


if __name__ == "__main__":
    main()
