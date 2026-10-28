"""Hardware configuration for the AGV.

Every pin number and tuning constant lives here so that a wiring change never
requires editing the logic.  Values marked CHECK must be confirmed against the
team's interface board before the first powered test.

Pin numbers are GPxx numbers (machine.Pin), not the jumper header numbers
printed on the PicoBot - see the PicoBot pinout drawing for the mapping.
"""

# ---------------------------------------------------------------- start/stop
BUTTON_PIN = 20             # CHECK - GP12, free pin on the lower header
BUTTON_ACTIVE_HIGH = True   # CHECK - True if the module pulls the line HIGH
BUTTON_DEBOUNCE_MS = 30     # contact bounce window for the push button module

# -------------------------------------------------------------------- lights
# LEDs are driven through the opto-isolator / transistor stage, never directly
# from the Pico (M1 build spec).
AMBER_LED_PIN = 26          # CHECK - GP28, flashing beacon while moving
AMBER_FLASH_MS = 250        # 250 ms on / 250 ms off = 2 Hz, within the range
                            # the human eye reads as a hazard beacon

# ------------------------------------------------------------- drive motors
# Motor Driver 2 (12 V rail) carries the two drive motors.
LEFT_MOTOR_DIR_PIN = 4      # Motor 3 = Motor Driv2 #1, GP4/GP5
LEFT_MOTOR_PWM_PIN = 5
RIGHT_MOTOR_DIR_PIN = 7     # Motor 4 = Motor Driv2 #2, GP6/GP7
RIGHT_MOTOR_PWM_PIN = 6

# ---------------------------------------------------------------- main loop
LOOP_PERIOD_MS = 5          # target main-loop period; keeps the button poll
                            # rate high and the line follower responsive

# -------------------------------------------------------------- line sensors
# Up to four sensors, the maximum allowed. The inner pair straddles the line
# and is all that line following needs. The outer pair looks for junction
# markings and is only used by navigation.
#
# Week 1 fits the inner pair only, so TURN_TRACKERS_FITTED is False and the
# outer pins are never configured. Set it True once the other two sensors are
# mounted - nothing else changes.
TURN_TRACKERS_FITTED = True

LINE_TRACKER_L_PIN = 18     # CHECK - inner left
LINE_TRACKER_R_PIN = 19     # CHECK - inner right
TURN_TRACKER_L_PIN = 22     # CHECK - outer left,  ignored while unfitted
TURN_TRACKER_R_PIN = 17     # CHECK - outer right, ignored while unfitted

# The PicoBot guide specifies TTL HIGH over white, LOW over black. Flip this
# if the bench test reports the opposite - the AGV would otherwise steer away
# from the line instead of towards it.
LINE_WHITE_IS_HIGH = True   # CHECK with test_line_sensors.py

# ----------------------------------------------------------- line following
# The steering correction has to scale with speed: too little and the AGV
# runs wide on bends, too much and it saws across the line. tools/tune.py
# sweeps both and reports how far the AGV strays. A ratio of roughly
# steer = speed / 3 tracked cleanly across the whole speed range; 80/30 was
# the fastest pair that still held the line in simulation, worth trying once
# 60/20 is working on the real table.
FOLLOW_SPEED = 100           # cruise speed, 0-100
FOLLOW_STEER = 50           # differential correction applied when off centre
RECOVER_SPEED = 100          # reverse speed when the line is lost
LOST_TIMEOUT_MS = 2000      # stop after this long with no line at all;
                            # set to 0 to reverse indefinitely

# --------------------------------------------------------------- route turns
TURN_SPEED = 100             # in-place turn speed, 0-100; tune on the robot
TURN_TIMEOUT_MS = 6000      # stop and report an error if the line is not found

TURN_DRIVE_OVER_MS = 550        # drive past the junction before rotating
TURN_DRIVE_OVER_SHORT_MS = 350  # at the junction just after a collect bay,
                                # so the AGV doesn't hit the wall
UTURN_DRIVE_OVER_MS = 700       # drive forward before a U-turn spin
UTURN_SPIN_MS = 1200            # spin blind before looking for the line

# ------------------------------------------------------------ grabber/lifter
SERVO_LIFTER_PIN = 13       # servo 2
SERVO_GRABBER_PIN = 15      # servo 1

# Servo angles in degrees. Note set_Grabber_angle clamps to 10-70 and
# set_Lifter_angle to 0-90.
GRABBER_OPEN = 70           # released, ready to take or drop a box
GRABBER_CLOSED = -10         # gripping a box

LIFTER_DOWN = 0             # ready to pick up
LIFTER_CARRY = 20           # box just off the ground
LIFTER_SHELF = 45           # delivery shelf height, same at every bay

# -------------------------------------------------------------- junctions
# A junction marking has to be visible for CONFIRM_MS before it counts, and
# the turn trackers have to be clear for CLEAR_MS before the next one can.
# Together these stop one physical junction being counted many times as it
# passes under the sensor, which would desynchronise the whole route.
JUNCTION_CONFIRM_MS = 20     # CHECK on the table - raise if flicker counts
JUNCTION_CLEAR_MS = 300     # CHECK on the table - must exceed the time the
                            # marking spends under the sensor at FOLLOW_SPEED

# ------------------------------------------------------------- navigation
# The initial node and direction are the start point for the route.
INITIAL_NODE = "HOME"        # CHECK - the node at the start of the route
INITIAL_DIRECTION = "N"     # CHECK - the direction the AGV faces at the start
# The typed M1 route. No longer driven - nav/mission.py plans the route now.
SEQUENCE = ["HOME", "RH", "RC", "R1", "R1_BAY", "R1", "R2", "RB1", "RB2", "RB2_BAY", "RB2", "RB1", "R2", "R1", "RC", "RH", "HOME"]

# ---------------------------------------------------------------- mission
FIRST_PICKUP = "R2_BAY"     # first far pickup bay to head for (R2_BAY or L2_BAY)
HOME_NODE = "HOME"          # where the run finishes (+15 with >= 2 delivered)
RUN_LIMIT_MS = 5 * 60 * 1000    # the run is five minutes
CYCLE_RESERVE_MS = 90 * 1000    # CHECK - time one collect + deliver + drive
                                # home; no new pickup starts with less left
EMPTY_BAYS_BEFORE_HOME = 4  # pickup bays found empty in a row before going
                            # home; 4 = every bay gets one try

# --------------------------------------------------------------- magnet pole
# One analogue Hall sensor at the gripper, read once a box is gripped.
# GP26-28 are the only ADC pins; GP26 is the beacon.
MAGNET_ADC_PIN = 27         # CHECK - GP28 (ADC2)
MAGNET_NORTH_IS_HIGH = True # CHECK with test_magnet.py - depends on mounting
MAGNET_DEADBAND = 2000      # CHECK - ADC counts either side of idle that
                            # count as a pole rather than noise
MAGNET_CALIBRATE_SAMPLES = 32
MAGNET_POLL_MS = 20
MAGNET_CONFIRM_READS = 3    # identical reads before a pole is believed
MAGNET_READ_TIMEOUT_MS = 1000   # CHECK - give up on the pole after this

# ------------------------------------------------------------ pole lamps
# Steady red for north, green for south. GP12/GP14 are plain-GPIO
# alternatives if GP27 is wanted as an ADC.
RED_LED_PIN = 16            # CHECK
GREEN_LED_PIN = 21          # CHECK

# ------------------------------------------------- rear bay distance sensor
# One VL53L0X looking straight back. The AGV turns its back to a shelf bay
# and reverses up to it to measure, because the sensor's field of view is far
# too wide to pick one bay out from across the corridor.
DISTANCE_FITTED = True
DISTANCE_REAR = (0, 8, 9)       # CHECK - I2C0: GP8 SDA, GP9 SCL

# MEASURE with test_bay.py: the reading at the bay mouth with a package in
# the bay, and with it empty. The threshold goes between the two.
BAY_FULL_MM = 120           # CHECK - at or under this, a package is there
BAY_NEAR_READS = 2          # near readings to call a bay full
BAY_CONFIRM_READS = 3       # far readings to call a bay empty
BAY_TIMEOUT_MS = 400        # CHECK - how long to stand at the mouth waiting
                            # for a verdict; no verdict by then reads full

# The reverse-up-to-the-bay manoeuvre. The corridor has no line running the
# way the AGV has to face, so these rotations are timed rather than ended by
# the line trackers - CHECK all three on the table before a full run.
BAY_FACE_AWAY_MS = 420      # CHECK - rotate 90 deg, back towards the bay
BAY_SPIN_MS = 840           # CHECK - rotate 180 deg, front towards the bay
BAY_APPROACH_MS = 700       # CHECK - reverse from the corridor to the mouth
BAY_ACQUIRE_MS = 1200       # how long to creep forward looking for the spur
                            # line after the spin before calling it lost
BAY_REVERSE_SPEED = 60      # slower than FOLLOW_SPEED: this reverse is blind
