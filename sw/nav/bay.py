"""Tell an empty storage bay from a full one, while driving past it.

Works like nav/junction.py: call poll() every main-loop pass and it returns
one event per bay. The difference is when it looks.

The distance sensors sit level with the junction (line) sensors, so a
junction marking means a bay is beside us right now. main() calls start()
there, and the measurement runs over the next few passes while the robot
keeps driving - it is never stopped to measure. Measuring the whole way
down the corridor instead would read the rack's own dividers as full bays.

    junction detected  ->  bay.start(side)
    next few passes    ->  bay.poll() returns EMPTY or FULL
    EMPTY              ->  turn in
    FULL               ->  carry on to the next junction

Deciding the verdict, as asked: enough far readings in a row means empty,
but a single near reading means full and ends the measurement there. A box
is the only thing that should be close, so one sighting is enough; being
wrong about "full" costs one bay, being wrong about "empty" drops a package
on top of another.
"""

from utime import ticks_ms, ticks_diff

NONE = 0
EMPTY = 1
FULL = 2

LEFT = "left"
RIGHT = "right"

_NAMES = {NONE: "none", EMPTY: "empty", FULL: "full"}


def name(result):
    return _NAMES.get(result, "?")


class BayDetector:
    """Args:
        left, right: distance sensors, each with read() -> mm or None
        full_mm:     at or under this, something is in the bay
        confirm_reads: far readings needed before calling a bay empty
        timeout_ms:  give up after this long and report full
    """

    def __init__(self, left, right, full_mm, confirm_reads, timeout_ms):
        self._sensors = {LEFT: left, RIGHT: right}
        self._full_mm = full_mm
        self._confirm_reads = confirm_reads
        self._timeout_ms = timeout_ms

        self._side = None           # None when not measuring
        self._far = 0
        self._since_ms = 0
        self._result = NONE
        self.count = 0              # bays measured since the last reset

    @property
    def measuring(self):
        return self._side is not None

    @property
    def side(self):
        return self._side

    def start(self, side):
        """Begin measuring the bay on `side`. Call when a junction fires."""
        if side not in self._sensors:
            raise ValueError("Unknown side: {}".format(side))
        self._side = side
        self._far = 0
        self._since_ms = ticks_ms()
        self._result = NONE

    def cancel(self):
        """Abandon a measurement, e.g. when a turn starts."""
        self._side = None
        self._result = NONE

    def reset(self):
        self.cancel()
        self.count = 0

    def bay_is_empty(self):
        """True if the last finished measurement found an empty bay.

        Asked for by name so main() can read it directly; poll() returns the
        same answer as an event the moment it is decided.
        """
        return self._result == EMPTY

    def distance(self):
        """Last raw reading on the side being measured, for bench tests."""
        if self._side is None:
            return None
        return self._sensors[self._side].last_mm

    def poll(self):
        """Return EMPTY or FULL once per bay, otherwise NONE.

        Must be called every main-loop pass. Never blocks.
        """
        if self._side is None:
            return NONE

        mm = self._sensors[self._side].read()

        if mm is not None and mm <= self._full_mm:
            return self._finish(FULL)       # a box - decided, stop here

        if mm is not None:
            self._far += 1
            if self._far >= self._confirm_reads:
                return self._finish(EMPTY)

        if ticks_diff(ticks_ms(), self._since_ms) >= self._timeout_ms:
            # Not enough readings in time: sensor missing or too fast past
            # the bay. Call it full rather than risk stacking a package.
            return self._finish(FULL)

        return NONE

    def _finish(self, result):
        self._side = None
        self._result = result
        self.count += 1
        return result
