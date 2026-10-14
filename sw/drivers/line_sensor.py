"""Reflective line sensors.

The arena is white tape on a dark table, and the sensor modules output TTL
HIGH over white, LOW over black (PicoBot guide, line sensor specification).
Some modules are wired the other way round, so the polarity is configurable
rather than assumed - get it wrong and the AGV steers away from the line
instead of towards it.

Up to four sensors, the maximum the team is allowed. The outer pair is
optional: line following itself only needs the inner pair, so the week 1
two-sensor robot leaves the outer pair unfitted and the same code runs
unchanged. With no outer sensors the array reports no junction, ever, and
the follower falls through to plain line following on every pass.

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
        # Pull down so a pin whose sensor is unfitted, unplugged or broken
        # reads black rather than floating. Black is the safe answer for
        # both pairs: an inner sensor reporting black stops the AGV, an
        # outer one reporting black means "no junction here".
        self._pin = Pin(pin_no, Pin.IN, Pin.PULL_DOWN)
        self._white_is_high = white_is_high

    def read(self):
        """Return WHITE (1) or BLACK (0), corrected for module polarity."""
        raw = self._pin.value()
        if self._white_is_high:
            return raw
        return 1 - raw


class LineSensorArray:
    """The four sensors, grouped the way the follower thinks about them."""

    def __init__(self, line_l_pin, line_r_pin,
                 turn_l_pin=None, turn_r_pin=None, white_is_high=True):
        """Pass turn_l_pin and turn_r_pin as None when the outer pair is not
        fitted - the two-sensor line following robot."""
        self._line_l = LineSensor(line_l_pin, white_is_high)
        self._line_r = LineSensor(line_r_pin, white_is_high)
        self._turn_l = LineSensor(turn_l_pin, white_is_high) if turn_l_pin is not None else None
        self._turn_r = LineSensor(turn_r_pin, white_is_high) if turn_r_pin is not None else None

    def line_trackers(self):
        """(left, right) inner sensors.  (1, 1) means on track."""
        return self._line_l.read(), self._line_r.read()

    @property
    def has_turn_trackers(self):
        return self._turn_l is not None and self._turn_r is not None

    def turn_trackers(self):
        """(left, right) outer sensors.  (0, 0) means no junction here.

        Returns (0, 0) when the outer pair is not fitted, so callers need no
        special case: no junction is ever reported.
        """
        if not self.has_turn_trackers:
            return BLACK, BLACK
        return self._turn_l.read(), self._turn_r.read()

    def read_all(self):
        """(turn_l, line_l, line_r, turn_r) - handy for bench testing.

        The outer readings are BLACK when that pair is not fitted.
        """
        turn_l, turn_r = self.turn_trackers()
        return (turn_l, self._line_l.read(), self._line_r.read(), turn_r)
