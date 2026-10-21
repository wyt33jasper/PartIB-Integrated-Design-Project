"""Hardware configuration for the AGV.

Every pin number and tuning constant lives here so that a wiring change never
requires editing the logic.  Values marked CHECK must be confirmed against the
team's interface board before the first powered test.

Pin numbers are GPxx numbers (machine.Pin), not the jumper header numbers
printed on the PicoBot - see the PicoBot pinout drawing for the mapping.
"""

# ---------------------------------------------------------------- start/stop
BUTTON_PIN = 20             # GP20 - as wired for the M1 marking
BUTTON_ACTIVE_HIGH = True   # CHECK - True if the module pulls the line HIGH
BUTTON_DEBOUNCE_MS = 30     # contact bounce window for the push button module

# -------------------------------------------------------------------- lights
# LEDs are driven through the opto-isolator / transistor stage, never directly
# from the Pico (M1 build spec).
AMBER_LED_PIN = 26          # GP26 - as wired for M1. This moved off GP28,
                            # which TURN_TRACKER_R_PIN also claimed; the two
                            # fought over the pin and the beacon never lit.
AMBER_FLASH_MS = 250        # 250 ms on / 250 ms off = 2 Hz, within the range
                            # the human eye reads as a hazard beacon

# ------------------------------------------------------------- drive motors
# Motor Driver 2 (12 V rail) carries the two drive motors.
LEFT_MOTOR_DIR_PIN = 4      # Motor 3 = Motor Driv2 #1, GP4/GP5
LEFT_MOTOR_PWM_PIN = 5
RIGHT_MOTOR_DIR_PIN = 7     # Motor 4 = Motor Driv2 #2, GP6/GP7
RIGHT_MOTOR_PWM_PIN = 6

# Drive PWM frequency. 1 kHz is the department's figure and a safe default
# for the TB6612FNG, but it is worth tuning: too low and the motors whine
# audibly at walking pace, too high and they lose torque at low duty, which
# shows up as the AGV refusing to creep.
MOTOR_PWM_FREQ_HZ = 1000    # CHECK - listen to it at FOLLOW_SPEED

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
TURN_TRACKER_L_PIN = 27     # CHECK - outer left,  ignored while unfitted
TURN_TRACKER_R_PIN = 28     # CHECK - outer right, ignored while unfitted

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
FOLLOW_SPEED = 50           # cruise speed, 0-100; measured on the table
FOLLOW_STEER = 20           # differential correction applied when off centre
RECOVER_SPEED = 100          # reverse speed when the line is lost
LOST_TIMEOUT_MS = 2000      # stop after this long with no line at all;
                            # set to 0 to reverse indefinitely

# --------------------------------------------------------------- route turns
# A turn happens in three phases, measured on the table during M1.
#
#   1. drive over    carry the wheels past the junction, so the AGV pivots
#                    beyond the marking rather than on top of it
#   2. commit        rotate blind for a moment, so the inner trackers clear
#                    the line being left; without this the turn "finishes"
#                    instantly on the line it has not left yet
#   3. seek          keep rotating until the line is reacquired
#
# These were fixed sleeps during M1, which left the stop button unpolled for
# up to 4.4 seconds. They are now phase deadlines inside the turn loop, so the
# motion is identical but the button is still read every pass.
TURN_DRIVE_OVER_MS = 1200   # measured on the table
TURN_COMMIT_MS = 300        # measured on the table
UTURN_DRIVE_OVER_MS = 1400  # measured on the table
UTURN_COMMIT_MS = 3000      # measured on the table

# Reversing back out of a dead-end bay once the line runs out.
BAY_REVERSE_MS = 3000       # measured on the table

TURN_SPEED = 60             # in-place turn speed, 0-100; measured on the table

# The wheels counter-rotate at this fraction of TURN_SPEED. Found on the
# table during M1: full speed overshot the line on every turn. Lower it if
# the AGV still swings past the new line, raise it if a turn times out.
TURN_ROTATE_SCALE = 0.8     # measured on the table
TURN_TIMEOUT_MS = 6000      # stop and report an error if the line is not found.
                            # Must exceed DRIVE_OVER_MS + COMMIT_MS below, or a
                            # turn times out before it has begun looking.

# -------------------------------------------------------------- junctions
# A junction marking has to be visible for CONFIRM_MS before it counts, and
# the turn trackers have to be clear for CLEAR_MS before the next one can.
# Together these stop one physical junction being counted many times as it
# passes under the sensor, which would desynchronise the whole route.
JUNCTION_CONFIRM_MS = 20    # measured on the table
JUNCTION_CLEAR_MS = 300     # measured on the table - must exceed the time the
                            # marking spends under the sensor at FOLLOW_SPEED

# ------------------------------------------------------------- navigation
# The initial node and direction are the start point for the route. The
# sequence is the list of nodes to visit in order
INITIAL_NODE = "HOME"        # CHECK - the node at the start of the route
INITIAL_DIRECTION = "N"     # CHECK - the direction the AGV faces at the start
# An explicit node list, which is what M1 ran. It is a demonstration route,
# not the finished system: once a package's pole is read, nav/destination.py
# and map.path_find() should generate the sequence instead of it being typed
# out here.
SEQUENCE = ["HOME", "RH", "RC", "R1", "R1_BAY", "R1", "R2", "RB1", "RB2",
            "RB2_BAY", "RB2", "RB1", "R2", "R1", "RC", "RH", "HOME"]

# --------------------------------------------------------------- magnet pole
# How the pole is read. The M1 wiring decides this, not preference.
#
#   "digital"   two unipolar Hall sensors facing opposite ways: one reads
#               high for north, the other for south, neither with no
#               package. Two plain GPIO pins, nothing to calibrate.
#   "analogue"  one ratiometric Hall sensor on an ADC pin. Better, because
#               it gives field strength rather than presence - but the Pico
#               has only GP26, GP27 and GP28, and M1 put the beacon on GP26
#               and the turn trackers on GP27 and GP28. No ADC is free.
#
# To switch to analogue, move one turn tracker to a plain GPIO (GP12 and
# GP14 are spare), set MAGNET_ADC_PIN to the pin that frees, and set
# MAGNET_MODE to "analogue". Nothing above the probe changes.
MAGNET_MODE = "digital"

# digital mode. These moved off GP10 and GP11, which I2C1 now needs for the
# right-hand bay rangefinder. GP12 and GP14 were the spare plain GPIO.
#
# The pair sits at the gripper, not on the sides of the chassis: a unipolar
# Hall only answers for the pole it faces, so a north-tuned sensor looking
# sideways would read every south-pole package on the lower racks as an
# empty bay. Bay occupancy is measured by the rangefinders instead, and
# these two do what they are good at - telling the poles apart, and saying
# when a package is close enough to grip, because neither reads high until
# the magnet is in range.
MAGNET_NORTH_PIN = 12       # CHECK - GP12, Hall sensor facing north pole
MAGNET_SOUTH_PIN = 14       # CHECK - GP14, Hall sensor facing south pole
MAGNET_ACTIVE_HIGH = True   # CHECK - True if the module pulls the line HIGH

# analogue mode - unusable until an ADC pin is freed, hence None
MAGNET_ADC_PIN = None
MAGNET_DEADBAND = 2000      # ADC counts either side of the idle reading that
                            # count as a pole rather than noise
MAGNET_NORTH_IS_HIGH = True # which way the sensor faces decides this
MAGNET_CALIBRATE_SAMPLES = 32

MAGNET_POLL_MS = 20
MAGNET_CONFIRM_READS = 3    # a wrong pole is +5 instead of +15

# ----------------------------------------------------------- pole indicator
# Steady red for a north pole package, steady green for south (M1 spec).
# Driven through the opto-isolator stage like every other LED.
RED_LED_PIN = 21            # CHECK - GP21
GREEN_LED_PIN = 22          # CHECK - GP22

# ------------------------------------------------------- bay rangefinders
# One side-facing rangefinder per side, for deciding whether a bay already
# holds a package before the AGV commits to turning into it. The spec says
# to "only deliver if the location is not already occupied", and the racks
# may be loaded before the run begins, so occupancy has to be measured.
#
#   "tof"      a VL53L0X on each side, reporting millimetres. The threshold
#              below is then a number to tune rather than a property of the
#              part, which is why this is the preferred option.
#   "digital"  a sensor in IO mode - the STU-22L - whose line goes active
#              inside its own fixed trip distance. One plain GPIO per side,
#              nothing on the bus, but no threshold to adjust.
#   "none"     not fitted. The scan then reports UNKNOWN for every bay, and
#              BAY_UNKNOWN_IS_OCCUPIED decides what the mission does about
#              it. main() prints a warning at start-up.
RANGER_MODE = "tof"         # a VL53L0X on each side, one per I2C bus

# Two VL53L0Xs both answer at 0x29, so they cannot share one bus without
# XSHUT sequencing at boot. They go on separate buses instead: I2C0 on
# GP16/GP17, I2C1 on GP10/GP11. Nothing else is on either bus now that the
# QR reader has gone, so there is no address to work around.
#
# A VL53L0X and a TMF8x01 (0x41) would share one bus happily, if the team
# ends up with one of each - which would free GP10/GP11 again.
RANGER_PINS = {
    "left":  {"i2c_id": 0, "sda": 16, "scl": 17, "address": 0x29, "pin": 8},
    "right": {"i2c_id": 1, "sda": 10, "scl": 11, "address": 0x29, "pin": 9},
}
RANGER_I2C_FREQ = 400000
RANGER_ACTIVE_HIGH = True   # CHECK - "digital" mode only

# VL53L0X laser pulse periods, in PCLKs, as the department's example sets
# them. Longer periods reach further and tolerate ambient light better at
# the cost of measurement time, so these are worth a try if an empty bay
# cannot be told from a full one. Valid pre-range values are 12 to 18 and
# final-range 8 to 14.
RANGER_VCSEL_PRE_RANGE = 18
RANGER_VCSEL_FINAL_RANGE = 14

# How often a sample is attempted. This must not be shorter than the
# sensor's own measurement time - about 30 ms for a VL53L0X - or most polls
# will find no new measurement ready and be discarded, and the scan will
# take proportionally longer to gather its samples.
RANGER_POLL_MS = 50         # comfortably longer than one measurement
RANGER_CONFIRM_READS = 3    # near readings needed before a bay is called
                            # occupied; one bad answer either stacks a
                            # package on top of another or skips a free bay

# A bay mouth is level with the junction, so by the time the outer trackers
# have confirmed the marking the rangefinder is only just reaching it. The
# AGV creeps forward for this long, sampling, before the scan is read.
#
# This must satisfy BAY_SCAN_ADVANCE_MS >= RANGER_POLL_MS *
# RANGER_CONFIRM_READS, or there is no time to gather the samples and every
# bay reads unknown. main() checks it at start-up.
BAY_SCAN_ADVANCE_MS = 400   # CHECK - measure where the sensor sits relative
                            # to the trackers and how fast the AGV creeps

# MEASURE THESE ON THE TABLE before trusting the scan. Two numbers settle it:
# what the sensor reads with a package in the bay, and what it reads with the
# bay empty. The threshold goes between them, and the gap between them is how
# much margin the scan has.
BAY_OCCUPIED_MM = 120       # CHECK - at or under this means a package is there
BAY_MIN_MM = 20             # CHECK - below the sensor's crosstalk floor, so
                            # the reading is rejected rather than believed
BAY_MAX_MM = 1200           # beyond this there is nothing in range at all,
                            # which means an empty bay

# What to do with a bay the sensor could not read. True skips it, which risks
# passing over a free bay; False tries it, which risks stacking one package
# on another. Skipping is cheaper: a shelf has six bays and only some are
# full, so a single bad reading costs nothing.
BAY_UNKNOWN_IS_OCCUPIED = True

# ----------------------------------------------------------------- gripper
# The linear actuator has no position feedback, so a movement is a timed run
# - drive it for a measured duration, then cut it. MEASURE THESE.
GRIPPER_FITTED = False      # CHECK - set True once the mechanism is on
GRIPPER_DIR_PIN = 0         # Actuator 1 = Motor Driv1 #1, GP0/GP1
GRIPPER_PWM_PIN = 1
GRIPPER_CLOSE_MS = 2500     # CHECK - measured on the bench
GRIPPER_OPEN_MS = 2500      # CHECK - measured on the bench
GRIPPER_SPEED = 100
GRIPPER_CLOSED_IS_EXTENDED = True   # CHECK - which way grips
GRIPPER_PWM_FREQ_HZ = 1000  # CHECK - as MOTOR_PWM_FREQ_HZ, for the actuator

# How long to keep closing on a package while waiting for the Hall pair to
# confirm a pole. If it never confirms, the loading bay is empty or the
# package is out of reach; the AGV releases and moves on rather than
# stalling there for the rest of the run.
COLLECT_TIMEOUT_MS = 4000

# ------------------------------------------------------------- the mission
# Where the AGV starts and finishes. The scoring gives +15 for being back
# inside the start box at the end of the run, provided at least two packages
# were delivered - the same as one more correct delivery, so it is worth
# giving up a marginal fourth package for.
HOME_NODE = "HOME"

# The loading bays to work, in order. The spec puts packages on four of
# them; which two are on which side comes from nav/destination.py, not from
# here, so this list is only about the order they are visited in.
PICKUP_ORDER = ["R1_BAY", "R2_BAY", "L1_BAY", "L2_BAY"]

# The run is five minutes. CYCLE_RESERVE_MS is what must be left on the
# clock for the AGV to start another package, so it has to cover the whole
# of one: drive to the loading bay, collect, drive to the shelf, scan,
# deliver, and get home afterwards. Reserving only the drive home lets the
# AGV commit to a round trip it cannot finish.
#
# MEASURE THIS. A rough figure from the map: a full four-package run covers
# about 48 junction-to-junction moves, so one package is around twelve, plus
# the drive home. Time one complete cycle on the table and use that.
RUN_LIMIT_MS = 5 * 60 * 1000
CYCLE_RESERVE_MS = 90 * 1000    # CHECK - one whole collect-and-deliver cycle
                                # plus the drive home
