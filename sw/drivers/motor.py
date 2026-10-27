"""Single DC motor channel on a TB6612FNG driver.

Each channel takes a direction line and a PWM line.  Based on the Motor class
in the department's test_motor.py, with an explicit stop() so that the caller
never has to know that "stopped" means zero duty cycle.
"""

from machine import Pin, PWM

PWM_FREQ_HZ = 1000          # as in V3.5; nobody tunes this
_FULL_SCALE = 65535


class Motor:
    def __init__(self, dir_pin, pwm_pin, reverse=False):
        """reverse=True flips the sense of this channel.

        The two drive motors face opposite ways on the chassis, so one of them
        needs its direction inverted for "forward" to mean the same thing on
        both sides.  Setting it here keeps the sign juggling out of the
        steering code.
        """
        self._dir = Pin(dir_pin, Pin.OUT)
        self._pwm = PWM(Pin(pwm_pin))
        self._pwm.freq(PWM_FREQ_HZ)
        self._pwm.duty_u16(0)
        self._reverse = reverse

    def drive(self, speed):
        """Set speed in the range -100 (full reverse) to +100 (full forward)."""
        if speed > 100:
            speed = 100
        elif speed < -100:
            speed = -100

        forward = speed >= 0
        if self._reverse:
            forward = not forward

        self._dir.value(0 if forward else 1)
        self._pwm.duty_u16(int(_FULL_SCALE * abs(speed) / 100))

    def stop(self):
        self._pwm.duty_u16(0)
