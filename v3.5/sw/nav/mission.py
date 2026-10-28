"""Decide where the AGV goes next: collect, deliver, or go home.

The status is always one of three:

    COLLECTING   driving to the pickup target. Whether a package is really
                 there is settled by closing the gripper and seeing whether
                 the Hall sensor finds a pole - there is no sensor that can
                 see into a pickup bay from the corridor.
    DELIVERING   driving the shelf corridor for the box being carried, backing
                 up to each bay in turn and taking the first one the rear
                 sensor reads as empty.
    RETURNING    driving home to finish the run.

The pickup target steps through PICKUP_ORDER: on to the next bay both when a
package is taken, so the load is spread, and when a bay turns out to be
empty, so a missing package does not stall the run.

Where a box goes is fixed by the task: the side it was collected from picks
the rack (R bays -> R shelves, L -> L) and its pole picks the deck (north ->
upper U shelves, south -> bottom B shelves). If a whole shelf reads full -
which should only happen through bad readings - the other deck of the same
rack is tried, then the box is carried home.

Nothing here touches hardware. It decides and replans `state`; main.py
drives the AGV and reports back what happened.
"""

from utime import ticks_ms, ticks_diff

from .map import JUNCTIONS, route

COLLECTING = "collecting"
DELIVERING = "delivering"
RETURNING = "returning"

COLLECT_BAYS = ("R1_BAY", "R2_BAY", "L1_BAY", "L2_BAY")    # the pickup bays

# The order pickup bays are tried in. Far bay on each side first, so the near
# bays stay clear for as long as possible.
PICKUP_ORDER = ("R2_BAY", "L2_BAY", "R1_BAY", "L1_BAY")

_DECK = {"north": "U", "south": "B"}    # pole (as drivers.magnet names it) -> shelf deck letter
_OTHER_DECK = {"U": "B", "B": "U"}      # fallback deck when a whole shelf reads full
_BAYS_PER_SHELF = 6                     # RB1..RB6, LU1..LU6, ...


def _corridor_node(bay):
    """The junction a bay hangs off: its only exit on the map."""
    for node in JUNCTIONS[bay].values():   # the bay's N, E, S, W neighbours
        if node is not None:
            return node                     # the one real exit, e.g. "RB3"


class Mission:
    def __init__(self, state, first_pickup, home, run_limit_ms,
                 cycle_reserve_ms, empty_bays_before_home):
        if first_pickup not in PICKUP_ORDER:
            raise ValueError(
                "First pickup must be one of {}, not {}".format(
                    PICKUP_ORDER, first_pickup))
        self.state = state                          # nav.navigator.State, replanned here
        self.pickup_target = first_pickup           # pickup bay being headed for
        self._home = home                           # node to finish at
        self._run_limit_ms = run_limit_ms           # length of the run
        self._cycle_reserve_ms = cycle_reserve_ms   # time one collect-deliver-home takes
        self._empty_bays_before_home = empty_bays_before_home  # empty pickups in a row before giving up

        self.status = COLLECTING        # COLLECTING / DELIVERING / RETURNING
        self.carrying = None            # (pole, "R" or "L") while a box is held
        self.delivered_count = 0        # boxes put on shelves this run
        self._deck = None               # "U" or "B" for the shelf being driven
        self._tried_other_deck = False  # True once the fallback deck is in use
        self._shelf_bays = ()           # the six bays of the shelf being driven
        self._empty_bays = 0            # pickup bays found empty in a row
        self._start_ms = ticks_ms()     # reset by start()

    def start(self):
        """Begin the run: plan the first collection from where the AGV stands."""
        self._start_ms = ticks_ms()
        self._collect_next()

    def elapsed_ms(self):
        return ticks_diff(ticks_ms(), self._start_ms)

    def bay_here(self):
        """(side, bay) for a shelf bay worth measuring here, else (None, None).

        side is "left" or "right" - which way the bay hangs off this junction,
        so main.py knows which way to turn its back to it.

        Only shelf bays are measured. Call before junction_turn(), while
        current_node is the junction the AGV is physically at.
        """
        if self.status != DELIVERING:
            return None, None
        beside = self.state.bays_beside()   # {"left": bay or None, "right": bay or None}
        for side in ("left", "right"):
            if beside[side] in self._shelf_bays:
                return side, beside[side]   # only one side can be this shelf's
        return None, None

    def take_bay(self, bay):
        """The bay read empty - route into it to deliver."""
        self.state.replan([self.state.current_node, bay])

    def skip_bay(self):
        """The bay read full - carry on down the shelf, or give up on it."""
        if self.state.next_node() is None:
            self._ran_out()

    def collected(self, pole):
        """Report the grab in the pickup bay the AGV is in. None = nothing there."""
        if pole is None:
            self._empty_bays += 1
            self.pickup_target = self._next_pickup()
            self._collect_next()
            return
        self._empty_bays = 0
        self.pickup_target = self._next_pickup()
        self.carrying = (pole, self.state.current_node[0])   # "R" or "L"
        self._deck = _DECK[pole]
        self._tried_other_deck = False
        self._deliver()

    def delivered(self):
        """Report the box left in the shelf bay the AGV is in."""
        self.delivered_count += 1
        self.carrying = None
        self._collect_next()

    def report(self):
        """One line of status, for the console."""
        return "{} (next pickup {}) | at {} facing {} | carrying {} | {} delivered | {} s".format(
            self.status, self.pickup_target, self.state.current_node,
            self.state.current_direction,
            self.carrying[0] if self.carrying else "nothing",
            self.delivered_count, self.elapsed_ms() // 1000)

    def _next_pickup(self):
        """The next pickup bay in PICKUP_ORDER, wrapping round."""
        index = PICKUP_ORDER.index(self.pickup_target)
        return PICKUP_ORDER[(index + 1) % len(PICKUP_ORDER)]

    def _ran_out(self):
        """The shelf ended with no bay to take."""
        if not self._tried_other_deck:
            self._tried_other_deck = True
            self._deck = _OTHER_DECK[self._deck]
            self._deliver()
        else:
            self._return_home()

    def _collect_next(self):
        time_left_ms = self._run_limit_ms - self.elapsed_ms()   # before the run ends
        if (self._empty_bays >= self._empty_bays_before_home or
                time_left_ms < self._cycle_reserve_ms):
            self._return_home()
            return
        self.status = COLLECTING
        # Straight into the bay: nothing can see a package from the corridor,
        # so the only way to test a pickup bay is to drive in and grip.
        self.state.replan(route(self.state.current_node, self.pickup_target))

    def _deliver(self):
        side = self.carrying[1]             # "R" or "L" - picks the rack
        self._shelf_bays = tuple("{}{}{}_BAY".format(side, self._deck, n)
                                 for n in range(1, _BAYS_PER_SHELF + 1))
        corridor = [_corridor_node(bay) for bay in self._shelf_bays]  # e.g. RB1..RB6 or U1..U6
        self.status = DELIVERING
        self.state.replan(route(self.state.current_node, corridor[0]) + corridor[1:])

    def _return_home(self):
        self.status = RETURNING
        self.state.replan(route(self.state.current_node, self._home))
