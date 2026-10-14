"""Turn raw turn-tracker readings into discrete junction events.

Topological navigation counts junctions instead of measuring distance, which
is what makes it survive the four tables not being identical.  The cost is
that counting has to be exactly right: miss one junction, or count one twice,
and every instruction after it is applied in the wrong place.

Two things make naive counting fail:

  Dwell    The main loop runs every few milliseconds but a junction marking
           takes a couple of hundred milliseconds to pass under the sensor.
           Testing "is a marking visible?" each pass would count the same
           junction dozens of times.

  Bounce   The sensor flickers at the edge of the marking, so the reading
           can go white, black, white within a few milliseconds.

So detection is a small state machine.  A marking has to be visible for
confirm_ms before it counts, and the trackers have to be clear again for
clear_ms before another junction can be counted.  One physical junction
produces exactly one event.

The signature is accumulated across the confirm window rather than sampled
once, because the two outer sensors rarely reach a crossroads on the same
loop pass - one side can lead the other by several milliseconds.
"""

from utime import ticks_ms, ticks_diff

# Junction signatures.  LEFT and RIGHT are bit flags so BOTH is their union.
NONE = 0
LEFT = 1
RIGHT = 2
BOTH = LEFT | RIGHT

_NAMES = {NONE: "none", LEFT: "left", RIGHT: "right", BOTH: "crossroads"}


def name(signature):
    """Human readable signature, for logging and bench tests."""
    return _NAMES.get(signature, "?")


# Internal states.
_CLEAR = 0        # no marking in view, ready to detect
_CONFIRMING = 1   # marking seen, waiting to see if it persists
_COUNTED = 2      # event already emitted, waiting for the marking to pass


class JunctionDetector:
    def __init__(self, sensors, confirm_ms=20, clear_ms=200):
        self._sensors = sensors
        self._confirm_ms = confirm_ms
        self._clear_ms = clear_ms

        self._state = _CLEAR
        self._since_ms = ticks_ms()
        self._signature = NONE
        self.count = 0          # junctions seen since the last reset

    def reset(self):
        """Forget the count and any part-detected junction.

        Called at the start of a route, so instruction 1 applies to the first
        junction after the AGV sets off rather than to something left over
        from the previous leg.
        """
        self._state = _CLEAR
        self._since_ms = ticks_ms()
        self._signature = NONE
        self.count = 0

    def poll(self):
        """Return a signature once per physical junction, otherwise NONE.

        Must be called every main-loop pass.  Never blocks.
        """
        turn_l, turn_r = self._sensors.turn_trackers()
        seen = (LEFT if turn_l else NONE) | (RIGHT if turn_r else NONE)
        now = ticks_ms()

        if self._state == _CLEAR:
            if seen:
                self._state = _CONFIRMING
                self._since_ms = now
                self._signature = seen
            return NONE

        if self._state == _CONFIRMING:
            if not seen:
                # Flicker, not a junction.  Drop back without counting.
                self._state = _CLEAR
                self._since_ms = now
                self._signature = NONE
                return NONE

            # Keep collecting: the far sensor may arrive a few ms after the near one.
            self._signature |= seen

            if ticks_diff(now, self._since_ms) >= self._confirm_ms:
                self._state = _COUNTED
                self._since_ms = now
                self.count += 1
                return self._signature
            return NONE

        # _COUNTED - wait for the marking to pass completely.
        if seen:
            self._since_ms = now          # still over it, restart the clear timer
        elif ticks_diff(now, self._since_ms) >= self._clear_ms:
            self._state = _CLEAR
            self._signature = NONE
        return NONE
