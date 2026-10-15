"""Hardware configuration for the AGV.

Every pin number and tuning constant lives here so that a wiring change never
requires editing the logic.  Values marked CHECK must be confirmed against the
team's interface board before the first powered test.

Pin numbers are GPxx numbers (machine.Pin), not the jumper header numbers
printed on the PicoBot - see the PicoBot pinout drawing for the mapping.
"""

# ---------------------------------------------------------------- start/stop
BUTTON_PIN = 12             # CHECK - GP12, free pin on the lower header
BUTTON_ACTIVE_HIGH = True   # CHECK - True if the module pulls the line HIGH
BUTTON_DEBOUNCE_MS = 30     # contact bounce window for the push button module

# -------------------------------------------------------------------- lights
# LEDs are driven through the opto-isolator / transistor stage, never directly
# from the Pico (M1 build spec).
AMBER_LED_PIN = 28          # CHECK - GP28, flashing beacon while moving
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
TURN_TRACKERS_FITTED = False

LINE_TRACKER_L_PIN = 18     # CHECK - inner left
LINE_TRACKER_R_PIN = 19     # CHECK - inner right
TURN_TRACKER_L_PIN = 20     # CHECK - outer left,  ignored while unfitted
TURN_TRACKER_R_PIN = 21     # CHECK - outer right, ignored while unfitted

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
FOLLOW_SPEED = 60           # cruise speed, 0-100
FOLLOW_STEER = 20           # differential correction applied when off centre
RECOVER_SPEED = 35          # reverse speed when the line is lost
LOST_TIMEOUT_MS = 2000      # stop after this long with no line at all;
                            # set to 0 to reverse indefinitely

# -------------------------------------------------------------- junctions
# A junction marking has to be visible for CONFIRM_MS before it counts, and
# the turn trackers have to be clear for CLEAR_MS before the next one can.
# Together these stop one physical junction being counted many times as it
# passes under the sensor, which would desynchronise the whole route.
JUNCTION_CONFIRM_MS = 20    # CHECK on the table - raise if flicker counts
JUNCTION_CLEAR_MS = 200     # CHECK on the table - must exceed the time the
                            # marking spends under the sensor at FOLLOW_SPEED
