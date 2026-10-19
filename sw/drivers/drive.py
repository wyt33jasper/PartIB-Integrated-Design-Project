"""Two-wheel differential drive.

Wraps the pair of drive motors so the rest of the code asks for a travel
speed and a steering correction instead of two separate motor speeds.
"""

from drivers.motor import Motor


class Drive:
    def __init__(self, left_dir, left_pwm, right_dir, right_pwm,
                 reverse_left=False, reverse_right=True):
        self._left = Motor(left_dir, left_pwm, reverse=reverse_left)
        self._right = Motor(right_dir, right_pwm, reverse=reverse_right)

    def move(self, speed, steer=0):
        """speed: -100..100 forward/back.  steer: -100..100, positive = right.

        Steering is applied differentially: the inside wheel slows down and
        the outside wheel speeds up, so the AGV turns without changing its
        average speed much.
        """
        self._left.drive(speed + steer)
        self._right.drive(speed - steer)

    # Rotating wheels oppose each other at this fraction of TURN_SPEED. Found
    # on the table during M1: full speed overshot the line every time.
    ROTATE_SCALE = 0.8

    def rotate(self, l_or_r, speed):
        """Rotate in place.  l_or_r: -1 = left, +1 = right.  speed: 0..100.

        The wheels counter-rotate, so the AGV spins about its own centre.
        The earlier version drove one wheel and stopped the other, which
        pivoted about that wheel and swung the body through a much wider
        arc than the table has room for.
        """
        self._left.drive(l_or_r * speed * Drive.ROTATE_SCALE)
        self._right.drive(-l_or_r * speed * Drive.ROTATE_SCALE)

    def u_turn(self,speed):

        self._left.drive(speed)
        self._right.drive(-speed)

    def stop(self):
        """Cut drive to both motors.  Safe to call at any time, repeatedly."""
        self._left.stop()
        self._right.stop()
