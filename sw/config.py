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
RIGHT_MOTOR_DIR_PIN = 6     # Motor 4 = Motor Driv2 #2, GP6/GP7
RIGHT_MOTOR_PWM_PIN = 7

# ---------------------------------------------------------------- main loop
LOOP_PERIOD_MS = 5          # target main-loop period; keeps the button poll
                            # rate high and the line follower responsive
