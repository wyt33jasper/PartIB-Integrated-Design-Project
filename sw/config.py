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