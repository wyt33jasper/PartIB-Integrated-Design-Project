"""Follow the line using only the inner line-tracker sensors.

The follower applies one steering correction per tick and does not make
route decisions or perform junction turns. Junction detection and route
planning are handled separately.
"""

from utime import ticks_ms, ticks_diff


class LineFollower:
    def __init__(self, drive, sensors,
                 speed=60, steer=40, reverse_speed=35,
                 lost_timeout_ms=2000):
        self._drive = drive
        self._sensors = sensors
        self._speed = speed
        self._steer = steer
        self._reverse_speed = reverse_speed
        self._lost_timeout_ms = lost_timeout_ms
        self._lost_since_ms = None

    def tick(self):
        """Read the inner trackers and apply one line-following correction."""
        line_l, line_r = self._sensors.line_trackers()

        if line_l and line_r:
            self._on_line()
            self._drive.move(self._speed, 0)
        elif not line_l and line_r:
            self._on_line()
            self._steer_right()
        elif line_l and not line_r:
            self._on_line()
            self._steer_left()
        else:
            self._line_lost()

    def _steer_right(self):
        self._drive.move(self._speed, self._steer)

    def _steer_left(self):
        self._drive.move(self._speed, -self._steer)

    def _on_line(self):
        self._lost_since_ms = None

    def _line_lost(self):
        """Reverse briefly to recover the line, then stop if it stays lost."""
        now = ticks_ms()
        if self._lost_since_ms is None:
            self._lost_since_ms = now

        if (self._lost_timeout_ms and
                ticks_diff(now, self._lost_since_ms) >= self._lost_timeout_ms):
            self._drive.stop()
            return

        self._drive.move(-self._reverse_speed, 0)

    def stop(self):
        self._drive.stop()
        self._lost_since_ms = None
