"""Line following.

STUB - the real controller cannot be written until the sensor array is fixed:
how many line sensors there are, how far apart they sit, and which side of the
line each one looks at.  Those answers decide whether this is a two-sensor
bang-bang controller, a three-sensor one with a centre reference, or a
four-sensor weighted-error PID.

What is already settled is the shape of the interface, so main.py does not
have to change when the controller lands:

    tick() is called once per main-loop pass and must return immediately.
    It reads the sensors, computes one correction, applies it, and returns.
    It never loops and never sleeps - the stop button is checked between
    every pair of calls.
"""


class LineFollower:
    def __init__(self, drive, speed=60):
        self._drive = drive
        self._speed = speed

    def tick(self):
        """Advance the controller by one step.  Must not block."""
        # TODO: read the line sensor array, compute the steering error and
        # call self._drive.move(self._speed, steer).
        #
        # Until the sensors are wired, hold the motors off so that running
        # main.py on the bench cannot drive the AGV off the table.
        self._drive.stop()

    def stop(self):
        self._drive.stop()
