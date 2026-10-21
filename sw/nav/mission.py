"""The semi-automatic delivery mission.

What makes this semi-automatic rather than scripted: nothing here names a
destination. M1 ran a route typed out in config.py, one junction at a time,
which worked once and then had to be retyped for every change of plan. This
plans each leg from a goal instead.

Three things are measured rather than assumed:

    which rack      from which loading bay the package was collected at,
                    which the AGV knows because it drove there
    which deck      from the magnet pole, read at the gripper
    which bay       from the side rangefinders, read from the corridor

and the fourth - the route to get there - is computed from the map.

The delivery leg is the part worth explaining. The AGV does not pick a bay
number in advance, because it cannot know which bays are already full; the
spec says to "only deliver if the location is not already occupied", and the
racks may be loaded before the run starts. So it drives the corridor that
serves the destination shelf and, at each junction, asks the rangefinder on
the relevant side whether that bay is occupied. The first free one gets the
package.

    corridor entry --+------------+------------+------------+--> shelf full
                     |            |            |            |
                  occupied     occupied       FREE          |
                  drive on     drive on      turn in        |
                                             deliver

The side to scan needs no table: the bay sits on whichever side the AGV
would turn to enter it, so State.turn_to(bay) answers both questions at
once. Where the corridor runs and which side its bays are on both come out
of the map, so moving a bay on the map moves it here too.

Replanning happens at every junction rather than once per leg. A
breadth-first search over sixty nodes is far cheaper than a main-loop
period, and it buys the property that matters on the table: if the AGV ends
up somewhere it did not intend to be, the next junction routes it onward
from where it actually is, instead of following a plan that no longer
describes reality.

ABOUT JUNCTION EVENTS

Navigation counts junction events, so it has to agree with the sensors about
which nodes produce one. A node is only visible to the outer trackers if a
branch leaves the line to one side - map.has_marking() decides this - so
nodes like UU and RH are driven straight over without an event, and the
planner walks the state through them rather than expecting one event per
map node. A bay produces no event either: the line simply runs out there,
which is why arrival at one is reported separately through at_bay().
"""

from utime import ticks_ms, ticks_diff

from drivers import rangefinder
from nav.destination import (LOADING_BAYS, shelf_for, side_of_loading_bay)
from nav.map import ahead, corridor_node, has_marking, is_dead_end, next_step
from nav.navigator import LEFT, RIGHT, STRAIGHT

# Phases. The AGV is always in exactly one of these.
TO_PICKUP = "to pickup"     # driving to a loading bay
COLLECT = "collect"         # in a loading bay, gripping and reading the pole
TO_SHELF = "to shelf"       # driving to the corridor serving the destination
SCAN = "scan"               # driving that corridor, testing each bay
DELIVER = "deliver"         # in a free bay, releasing
TO_HOME = "to home"         # driving back to the start box
FINISHED = "finished"

# What at_bay() asks the main loop to do with the gripper.
PICK_UP = "pick up"
PUT_DOWN = "put down"


class Mission:
    """Plans the run. Reads no pins and drives no motors.

    The main loop owns the hardware and reports events; this decides what
    to do about them. That split is what lets the whole mission be driven
    on a desktop, with no robot, by feeding it the same events.
    """

    def __init__(self, state, scanner, pickups=None, home=None,
                 run_limit_ms=0, cycle_reserve_ms=0,
                 unknown_is_occupied=True):
        self.state = state
        self._scanner = scanner
        self._pickups = list(pickups if pickups is not None
                             else sorted(LOADING_BAYS))
        for bay in self._pickups:
            if bay not in LOADING_BAYS:
                raise ValueError("Not a loading bay: {}".format(bay))

        self._home = home if home is not None else state.current_node
        self._run_limit_ms = run_limit_ms
        self._cycle_reserve_ms = cycle_reserve_ms
        self._unknown_is_occupied = unknown_is_occupied

        self.phase = TO_PICKUP
        self.carrying = None            # the pole of the package held
        self.delivered = 0
        self.collected_count = 0
        self.skipped = []               # bays found occupied, for the log

        self._target = None
        self._pickup = None             # the loading bay being worked
        self._shelf = None              # ((corridor, bay), ...) or None
        self._started_ms = None
        self._now_ms = ticks_ms()
        self._cycle_started_ms = None
        self.longest_cycle_ms = 0       # the slowest package so far

    # ------------------------------------------------------------ the clock

    def tick(self, now_ms=None):
        """Note the time. Call every main-loop pass; never blocks."""
        self._now_ms = ticks_ms() if now_ms is None else now_ms

    @property
    def elapsed_ms(self):
        if self._started_ms is None:
            return 0
        return ticks_diff(self._now_ms, self._started_ms)

    @property
    def reserve_ms(self):
        """What must be left on the clock to start another package.

        The configured figure is a guess made before the run. Once a
        package has actually been timed, the slowest one observed is a
        better estimate than any number typed in beforehand - it includes
        the real turn times, the real corridor lengths and however many
        occupied bays had to be driven past - so whichever is larger wins.

        This is what stops the AGV setting off on a round trip it cannot
        finish: a guess that is too small does it once, and then the
        measurement corrects it for the rest of the run.
        """
        return max(self._cycle_reserve_ms, self.longest_cycle_ms)

    @property
    def time_short(self):
        """True once there is no time for another package.

        This is only ever consulted at one moment - a bay has been dealt
        with and the AGV is choosing between another pickup and the drive
        home - so the reserve has to cover the whole of what it is deciding
        to start: collect, deliver, and get home afterwards. Reserving only
        the drive home lets the AGV commit to a round trip it cannot
        finish, and the five minutes runs out with it still on the table.

        It is not checked continuously. Abandoning a package half way
        through a delivery forfeits the fifteen points for it and gains
        nothing, so a leg once started is finished.
        """
        if not self._run_limit_ms:
            return False
        return self.elapsed_ms >= self._run_limit_ms - self.reserve_ms

    # ------------------------------------------------------------- what next

    @property
    def target(self):
        return self._target

    @property
    def expecting_bay(self):
        """True when losing the line means arrival rather than a fault.

        A bay is a spur with nothing past it, so the line runs out there.
        Everywhere else, two black inner trackers mean the AGV has strayed
        off the line and the follower should try to recover.

        Being bound for a bay is not enough: the AGV has to have turned into
        the spur. Otherwise a stray loss of line anywhere on the long drive
        to a loading bay would be read as having arrived at it.
        """
        if self._target is None or not is_dead_end(self._target):
            return False
        return (self.state.current_node == corridor_node(self._target) and
                ahead(self.state.current_node,
                      self.state.current_direction) == self._target)

    def start(self, now_ms=None):
        """Plan the first leg. Returns the turn to make where the AGV stands.

        The AGV may be parked facing the wrong way, so the first instruction
        is a turn on the spot rather than something to wait for a junction
        for.
        """
        self._started_ms = ticks_ms() if now_ms is None else now_ms
        self._now_ms = self._started_ms
        self._take_next_pickup()
        return self._depart()

    def at_junction(self):
        """The turn to make at the junction just detected.

        During the delivery leg a junction is only a position update - it
        says which bay the AGV is now beside. Whether to turn into that bay
        is not decided here; the side rangefinder decides that, on every
        pass, through bay_beside().

        Returns None when the run is over or the AGV is somewhere the map
        cannot route it out of - in both cases the main loop should stop.
        """
        passed = self._bay_here() if self.phase == SCAN else None

        self._walk_to_here()

        # The corridor entrance is the first bay, so the scan goes live as
        # soon as the AGV stands beside one. The target is the far end of
        # the corridor, which it only reaches if every bay was full, so
        # arriving at the target cannot be what starts the scan.
        if self.phase == TO_SHELF and self._bay_here() is not None:
            self.phase = SCAN

        if passed is not None and self._bay_here() != passed:
            # Driven past it without the sensor ever reporting it free, so
            # it held a package. Recorded for the log, not for the plan.
            self.skipped.append(passed)

        if self.phase == SCAN and self._bay_here() is not None:
            # Arriving beside a new bay. Clear the window, or the verdict
            # would be answered from the approach - where the sensor was
            # pointed down an empty corridor and everything read far. That
            # is not a pause: the AGV keeps driving, and bay_beside() stays
            # silent for the confirm_reads * RANGER_POLL_MS it takes to
            # gather samples of this bay, which is a couple of centimetres
            # of travel.
            self._scanner.arm()

        if self.state.current_node == self._target:
            if not self._target_reached():
                return None

        return self._depart()

    def bay_beside(self):
        """The turn into the bay beside the AGV, if the sensor says it is free.

        Called every main-loop pass while the AGV is running a corridor.
        Returns a turn as soon as the rangefinder on the relevant side
        reports an empty bay, and None otherwise - so the AGV drives on
        past the full ones without stopping at any of them.

        This is the whole of the bay choice. The junction count says which
        bay the AGV is beside; the sensor says whether to take it.
        """
        if self.phase != SCAN:
            return None

        bay = self._bay_here()
        if bay is None:
            return None             # between bays, nothing to look into

        side = self.state.turn_to(bay)
        if side not in (LEFT, RIGHT):
            raise RuntimeError(
                "{} is {} of {}, not to one side".format(
                    bay, side, self.state.current_node))

        if not self._is_free(side):
            return None

        self.phase = DELIVER
        self._target = bay
        return self.state.face(bay)

    def _bay_here(self):
        """The bay beside the AGV right now, or None if it is between bays.

        Read off the shelf rather than counted into an index of its own, so
        there is no separate tally that can disagree with where the AGV
        thinks it is.
        """
        if self._shelf is None:
            return None
        for corridor, bay in self._shelf:
            if corridor == self.state.current_node:
                return bay
        return None

    def _is_free(self, side):
        """Whether the sensor on `side` is reporting a free bay."""
        verdict = self._scanner.occupied(side)
        if verdict is rangefinder.EMPTY:
            return True
        # A bay the sensor could not read is skipped by default. Skipping
        # costs the width of a junction; stacking one package on another
        # costs the package.
        return (verdict is rangefinder.UNKNOWN
                and not self._unknown_is_occupied)

    def at_bay(self):
        """What to do at the bay the AGV has just driven into.

        Returns PICK_UP, PUT_DOWN, or None if this dead end was not planned.
        """
        bay = self._target
        if bay is None or not is_dead_end(bay):
            return None

        # The bay produces no junction event, so the state is still at the
        # corridor junction; record the drive in.
        if self.state.current_node != bay:
            self.state.arrive(bay)

        if self.phase == TO_PICKUP:
            self.phase = COLLECT
            return PICK_UP
        if self.phase == DELIVER:
            return PUT_DOWN
        return None

    def collected(self, pole):
        """Report the gripper closed, with the pole read - or None if empty.

        A loading bay can be empty, and the pole can fail to confirm. Both
        come back as None, and both mean the same thing to the plan: there
        is nothing to deliver from here, so move on.
        """
        if self.phase != COLLECT:
            raise RuntimeError("collected() in phase {}".format(self.phase))

        self.carrying = pole
        if pole is None:
            self._after_bay()
            return

        self.collected_count += 1
        self._shelf = shelf_for(pole, side_of_loading_bay(self._pickup))
        self.phase = TO_SHELF
        # Aim at the far end of the corridor. The AGV is not trying to
        # reach it - the sensor will normally stop it at the first free bay
        # along the way - but it is where the drive ends if every bay on
        # the shelf turns out to be full.
        self._target = self._shelf[-1][0]
        # Readings taken on the way here were of something else entirely.
        self._scanner.arm()

    def dropped(self):
        """Report the gripper opened and the package left in the bay."""
        if self.phase != DELIVER:
            raise RuntimeError("dropped() in phase {}".format(self.phase))
        self.delivered += 1
        self.carrying = None
        self._shelf = None
        self._after_bay()

    def leaving_bay(self):
        """The turn to make having reversed back out onto the corridor.

        Reversing is not a route step the map describes, so it is recorded
        here: the AGV ends up at the corridor junction, still facing into
        the bay it has left.
        """
        bay = self.state.current_node
        if not is_dead_end(bay):
            raise RuntimeError("Not in a bay: {}".format(bay))
        self.state.reverse_to(corridor_node(bay))
        return self._depart()

    # ------------------------------------------------------------- internals

    def _walk_to_here(self):
        """Advance the state to the junction the AGV has just reached.

        One event does not mean one map node: the nodes between here and the
        next marking are driven straight over, so they are consumed now.
        """
        while True:
            if self._target is None:
                return
            node = next_step(self.state.current_node, self._target)
            if node is None:
                return                      # already standing on the target
            if is_dead_end(node):
                # The line runs out in a bay, so arrival there is reported by
                # at_bay() and not by a junction event.
                return
            self.state.arrive(node)
            if node == self._target:
                return
            if has_marking(node, self.state.current_direction):
                return

    def _depart(self):
        """Face the next node on the way to the target. Returns the turn."""
        if self._target is None:
            return None
        node = next_step(self.state.current_node, self._target)
        if node is None:
            return STRAIGHT
        return self.state.face(node)

    def _target_reached(self):
        """Advance the phase on arriving at the target. False ends the run."""
        if self.phase == TO_SHELF:
            # Only reachable on a shelf one bay long, where the entrance
            # and the far end are the same junction.
            self.phase = SCAN
            return self._shelf_full() if not self._is_free(
                self.state.turn_to(self._bay_here())) else True
        if self.phase == SCAN:
            # The far end of the corridor, with the sensor never having
            # reported a free bay on the way up it. The line stops here, so
            # this is the moment to decide rather than drive on.
            return self._shelf_full()
        if self.phase == TO_HOME:
            self.phase = FINISHED
            self._target = None
            return False
        # TO_PICKUP and DELIVER both target a bay, which produces no
        # junction event; arriving at one goes through at_bay() instead.
        raise RuntimeError(
            "Reached {} in phase {} unexpectedly".format(self._target,
                                                         self.phase))

    def _shelf_full(self):
        """The corridor ran out with no free bay reported on either look.

        There is nowhere correct to put the package, and the wrong shelf
        scores less than not delivering at all once the penalty for a
        misplaced package is counted, so the AGV takes it home.
        """
        self.skipped.append(self._shelf[-1][1])
        self.phase = TO_HOME
        self._target = self._home
        return True

    def _after_bay(self):
        """Choose the next leg after a bay has been dealt with."""
        if self._cycle_started_ms is not None:
            took = ticks_diff(self._now_ms, self._cycle_started_ms)
            if took > self.longest_cycle_ms:
                self.longest_cycle_ms = took
            self._cycle_started_ms = None

        if self.time_short or not self._pickups:
            self.phase = TO_HOME
            self._target = self._home
            return
        self._take_next_pickup()

    def _take_next_pickup(self):
        if not self._pickups:
            self.phase = TO_HOME
            self._target = self._home
            return
        self._pickup = self._pickups.pop(0)
        self.phase = TO_PICKUP
        self._target = self._pickup
        self._cycle_started_ms = self._now_ms

    # ---------------------------------------------------------------- report

    def report(self):
        return ("{} -> {} (at {} facing {}, carrying {}, {} delivered, "
                "{:.0f}s elapsed, {:.0f}s reserved)").format(
                    self.phase, self._target, self.state.current_node,
                    self.state.current_direction, self.carrying or "nothing",
                    self.delivered, self.elapsed_ms / 1000,
                    self.reserve_ms / 1000)
