"""Reflective line sensors.

The arena is white tape on a dark table, and the sensor modules output TTL
HIGH over white, LOW over black (PicoBot guide, line sensor specification).
Some modules are wired the other way round, so the polarity is configurable
rather than assumed - get it wrong and the AGV steers away from the line
instead of towards it.

Four sensors, the maximum the team is allowed:

    turn L      line L   line R      turn R
      |           |        |           |
      |          [===== white line =====]
      |                                |
      sits on black while following     sits on black while following

The inner pair straddles the line and both read white while the AGV is on
track.  The outer pair reads black until a junction marking appears on one
side, which is what tells the AGV a turn is available.
"""

from machine import Pin

WHITE = 1
BLACK = 0


class LineSensor:
    def __init__(self, pin_no, white_is_high=True):
        self._pin = Pin(pin_no, Pin.IN)
        self._white_is_high = white_is_high

    def read(self):
        """Return WHITE (1) or BLACK (0), corrected for module polarity."""
        raw = self._pin.value()
        if self._white_is_high:
            return raw
        return 1 - raw


class LineSensorArray:
    """The four sensors, grouped the way the follower thinks about them."""

    def __init__(self, line_l_pin, line_r_pin, turn_l_pin, turn_r_pin,
                 white_is_high=True):
        self._line_l = LineSensor(line_l_pin, white_is_high)
        self._line_r = LineSensor(line_r_pin, white_is_high)
        self._turn_l = LineSensor(turn_l_pin, white_is_high)
        self._turn_r = LineSensor(turn_r_pin, white_is_high)

    def line_trackers(self):
        """(left, right) inner sensors.  (1, 1) means on track."""
        return self._line_l.read(), self._line_r.read()

    def turn_trackers(self):
        """(left, right) outer sensors.  (0, 0) means no junction here."""
        return self._turn_l.read(), self._turn_r.read()

    def read_all(self):
        """(turn_l, line_l, line_r, turn_r) - handy for bench testing."""
        return (self._turn_l.read(), self._line_l.read(),
                self._line_r.read(), self._turn_r.read())
