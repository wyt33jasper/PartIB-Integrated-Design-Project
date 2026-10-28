"""Tell an empty shelf bay from a full one with the rear distance sensor.

The sensor looks straight back, so main.py turns the AGV's back to the bay
and reverses up to it before measuring. Close range is the whole point: the
sensor's cone is wide, and only from the bay mouth does it see one bay
rather than the whole rack.

Works like nav/junction.py - call poll() every pass of the approach loop and
it returns one verdict per bay. There are two stages, because a far reading
only means something once the AGV has arrived:

    start(bay)   reversing towards the bay. A near reading is already a full
                 bay, and stops the reverse before it touches the package.
    settle()     stopped at the mouth. A run of far readings now means empty.
    give_up()    no clear reading in time - record full.

Full is the safe default either way: it skips the bay instead of stacking a
second package on top of the one already there.
"""

from utime import ticks_ms, ticks_diff

NONE = 0
EMPTY = 1
FULL = 2

_NAMES = {NONE: "none", EMPTY: "empty", FULL: "full"}


def name(result):
    return _NAMES.get(result, "?")


class BayDetector:
    """Args:
        sensor:        rear distance sensor, read() -> mm or None
        full_mm:       at or under this, something is in the bay
        near_reads:    near readings needed to call a bay full
        confirm_reads: far readings needed to call a bay empty
    """

    def __init__(self, sensor, full_mm, near_reads, confirm_reads):
        self._sensor = sensor
        self._full_mm = full_mm
        self._near_reads = near_reads
        self._confirm_reads = confirm_reads

        self.measuring = False      # True between start() and a verdict
        self.verdict = NONE         # the verdict this bay ended on
        self.bay = None             # bay being measured, for the console
        self.last_mm = None         # last reading taken
        self._near = 0              # readings at or under _full_mm
        self._far = 0               # readings over _full_mm
        self._settled = False       # True once the AGV has stopped at the bay

    def start(self, bay):
        """Begin measuring bay, as the reverse towards it starts."""
        self.measuring = True
        self.verdict = NONE
        self.bay = bay
        self.last_mm = None
        self._near = 0
        self._far = 0
        self._settled = False

    def settle(self):
        """The AGV has stopped at the bay mouth, so empty is now believable."""
        self._far = 0               # readings taken on the way in do not count
        self._settled = True

    def cancel(self):
        """Abandon a measurement, e.g. on a stop press."""
        self.measuring = False

    def give_up(self):
        """No clear reading in time. Record full so the bay is skipped."""
        self.measuring = False
        self.verdict = FULL
        return FULL

    def bay_is_empty(self):
        """True once this bay has been measured and found empty.

        Asked for by name, so main() can read the verdict without comparing
        against nav.bay.EMPTY itself.
        """
        return self.verdict == EMPTY

    def poll(self):
        """Return EMPTY or FULL once per bay, otherwise NONE. Never blocks."""
        if not self.measuring:
            return NONE

        mm = self._sensor.read()    # mm, or None if no new measurement yet
        if mm is None:
            return NONE

        self.last_mm = mm
        if mm <= self._full_mm:
            self._near += 1
            if self._near >= self._near_reads:
                return self._finish(FULL)
        elif self._settled:
            self._far += 1
            if self._far >= self._confirm_reads:
                return self._finish(EMPTY)
        return NONE

    def _finish(self, result):
        self.measuring = False
        self.verdict = result
        return result
