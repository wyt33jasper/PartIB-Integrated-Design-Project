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

try:
    from utime import ticks_ms, ticks_diff
except ImportError:
    # CPython has no utime module; monotonic time keeps detector tests
    # runnable on a desktop without changing MicroPython behavior.
    from time import monotonic

    def ticks_ms():
        return int(monotonic() * 1000)

    def ticks_diff(now, before):
        return now - before

# Public signatures returned by poll(). LEFT and RIGHT are bit flags rather
# than separate enum values, so OR-ing them together gives BOTH.
NONE = 0
LEFT = 1
RIGHT = 2
BOTH = LEFT | RIGHT

# Convert the public bitmask into a label suitable for status messages.
_NAMES = {NONE: "none", LEFT: "left", RIGHT: "right", BOTH: "crossroads"}


def name(signature):
    """Human readable signature, for logging and bench tests."""
    return _NAMES.get(signature, "?")


# Internal detector states:
# CLEAR      ready to begin checking a new marking
# CONFIRMING a marking has appeared and must persist for confirm_ms
# COUNTED    this marking already produced an event; wait for it to clear
_CLEAR = 0        # no marking in view, ready to detect
_CONFIRMING = 1   # marking seen, waiting to see if it persists
_COUNTED = 2      # event already emitted, waiting for the marking to pass


class JunctionDetector:
    """Turn outer turn-tracker readings into one event per junction.

    Args:
        sensors: Object providing turn_trackers(), returning (left, right)
            readings where a truthy value means that sensor sees the marking.
        confirm_ms: Continuous time a marking must be visible before it is
            reported. This filters out short sensor glitches.
        clear_ms: Time both sensors must be clear before another junction can
            be detected. This prevents one wide marking being counted twice.

    Call poll() frequently from the main loop. It does not wait for either
    timer; it returns NONE until the current state has made progress.
    """
    def __init__(self, sensors, confirm_ms=200, clear_ms=200):
        self._sensors = sensors
        self._confirm_ms = confirm_ms
        self._clear_ms = clear_ms

        # Start ready to detect. _since_ms is the start of the current
        # confirmation or clear interval, depending on _state.
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

    def pause(self):
        """Discard a partial detection and require a clear marking to re-arm.

        Call once when junction polling is suspended, such as while the AGV
        is turning. The count of already-detected junctions is preserved.
        Setting _since_ms to None means that the clear timer starts when
        polling resumes, rather than counting time spent paused as clear time.
        """
        self._state = _COUNTED
        self._since_ms = None
        self._signature = NONE

    def poll(self):
        """Return a signature once per physical junction, otherwise NONE.

        Must be called every main-loop pass.  Never blocks.
        """
        # Read both trackers on every poll. The signature records which
        # tracker(s) saw a marking during the confirmation window.
        turn_l, turn_r = self._sensors.turn_trackers()
        seen = (LEFT if turn_l else NONE) | (RIGHT if turn_r else NONE)
        
        now = ticks_ms()

        if self._state == _CLEAR:
            if seen != 0:
                # Start a fresh confirmation window at the first observation.
                # Do not report yet: the reading could be brief sensor noise.
                self._state = _CONFIRMING
                self._since_ms = now
                self._signature = seen
            return NONE

        if self._state == _CONFIRMING:
            if seen == 0:
                # Both trackers went clear before confirmation completed.
                # Treat that as flicker and require a fresh visible interval.
                self._state = _CLEAR
                self._since_ms = now
                self._signature = NONE
                return NONE

            # Accumulate instead of replacing the signature: at a crossroads,
            # one tracker may see the marking slightly before the other.
            self._signature |= seen

            # ticks_diff handles the embedded clock's counter wraparound.
            # Emit exactly one signature, then lock out detection until clear.
            if ticks_diff(now, self._since_ms) >= self._confirm_ms:
                self._state = _COUNTED
                self._since_ms = now
                self.count += 1
                print("signature:", self._signature)
                print("senn:", seen)
                return self._signature
            else:
                return NONE

        # _COUNTED: suppress further events while either tracker still sees
        # this marking. Each visible reading restarts the clear interval.
        if seen !=0:
            self._since_ms = now
        elif self._since_ms is None:
            # pause() was called while polling was suspended; begin timing now.
            self._since_ms = now
        elif ticks_diff(now, self._since_ms) >= self._clear_ms:
            # The marking has fully passed. Re-arm for the next junction.
            self._state = _CLEAR
            self._signature = NONE
        return NONE
