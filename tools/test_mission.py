"""Drive the whole mission on a desktop, against the real map.

    python3 tools/test_mission.py

Table below plays the two parts the mission cannot play for itself: the main
loop, which carries out the turns it asks for, and the table, which decides
what the sensors see. It walks the AGV node by node across nav/map.py and
reports an event exactly where the real sensors would - a junction only at a
node with a marking to one side, and a bay arrival only where the line runs
out.

It keeps its own idea of where the AGV physically is, independent of the
mission's, and checks the two agree after every event. That is the failure
this whole design is vulnerable to: a route that is one junction out of step
is wrong everywhere after that point, and it looks like a mechanical problem
on the table.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))
sys.path.insert(0, os.path.join(REPO, "sw"))

import utime                                                   # noqa: E402
from drivers import rangefinder                                # noqa: E402
from drivers.magnet import NORTH, SOUTH                        # noqa: E402
from nav import destination as dest                            # noqa: E402
from nav import mission as mis                                 # noqa: E402
from nav.map import (COMPASS, JUNCTIONS, ahead, corridor_node,  # noqa: E402
                     has_marking, is_dead_end)
from nav.mission import Mission                                # noqa: E402
from nav.navigator import State                                # noqa: E402

failures = []


def check(what, got, want):
    ok = got == want
    if not ok:
        failures.append("{} (got {!r}, wanted {!r})".format(what, got, want))
    print("  {}  {:<54} {!r}".format("ok  " if ok else "FAIL", what, got))


def ok(what, condition):
    check(what, bool(condition), True)


_OFFSET = {"straight": 0, "right": 1, "uturn": 2, "left": 3}


class FakeScanner:
    """Reports whatever is beside the AGV, as the real scanner would.

    The real one keeps a rolling window; what matters to the mission is
    only that occupied(side) describes where the AGV is now, so this
    reports the current surroundings directly.
    """

    fitted = True
    confirm_reads = 3
    poll_ms = 50
    window_ms = 250

    def __init__(self):
        self.reading = {rangefinder.LEFT: None, rangefinder.RIGHT: None}
        self.polls = 0

    def arm(self):
        self.reading = {rangefinder.LEFT: None, rangefinder.RIGHT: None}

    reset = arm

    def poll(self):
        self.polls += 1

    def occupied(self, side):
        return self.reading[side]


class Table:
    """The main loop and the table, as far as the mission can tell."""

    def __init__(self, start="HOME", heading="N", occupied=(), poles=None,
                 unreadable=(), **kw):
        self.scanner = FakeScanner()
        self.state = State(start, heading)
        self.mission = Mission(self.state, self.scanner, **kw)
        self.node = start                   # where the AGV physically is
        self.heading = heading
        self.occupied = set(occupied)       # bays that already hold a package
        self.unreadable = set(unreadable)   # bays the sensor cannot read
        self.poles = dict(poles or {})      # loading bay -> pole, or absent
        self.log = []
        self.desync = []
        self.distance = 0                   # nodes driven, as a cost measure

    # ------------------------------------------------------------- physical

    def _turn(self, turn):
        self.heading = COMPASS[
            (COMPASS.index(self.heading) + _OFFSET[turn]) % 4]

    def _drive(self):
        """Roll forward to the next thing the sensors would notice."""
        while True:
            nxt = ahead(self.node, self.heading)
            if nxt is None:
                return "lost"           # drove off the end of a line
            self.node = nxt
            self.distance += 1
            if is_dead_end(nxt):
                return "bay"            # the line runs out here
            if has_marking(nxt, self.heading):
                return "junction"

    def _present_bays(self):
        """Set the side readings for whatever is beside the AGV now."""
        index = COMPASS.index(self.heading)
        sides = ((rangefinder.LEFT, COMPASS[(index - 1) % 4]),
                 (rangefinder.RIGHT, COMPASS[(index + 1) % 4]))
        for side, compass in sides:
            neighbour = JUNCTIONS[self.node][compass]
            if neighbour is None or not is_dead_end(neighbour):
                self.scanner.reading[side] = None
            elif neighbour in self.unreadable:
                self.scanner.reading[side] = rangefinder.UNKNOWN
            elif neighbour in self.occupied:
                self.scanner.reading[side] = rangefinder.OCCUPIED
            else:
                self.scanner.reading[side] = rangefinder.EMPTY

    def _agree_node(self, where):
        """Where the AGV is must match. Checked as each event is reported,
        before the mission has been asked for a turn."""
        if self.node != self.state.current_node:
            self.desync.append(
                "{}: table is at {}, mission thinks {}".format(
                    where, self.node, self.state.current_node))

    def _agree_fully(self, where):
        """Position and heading must match once a turn has been carried out."""
        self._agree_node(where)
        if self.heading != self.state.current_direction:
            self.desync.append(
                "{}: table faces {}, mission thinks {}".format(
                    where, self.heading, self.state.current_direction))

    # ----------------------------------------------------------------- run

    def run(self, limit=400, run_ms_per_node=0):
        m = self.mission
        turn = m.start(now_ms=0)
        clock = 0
        self.log.append("start: {} -> {}".format(m.phase, turn))
        if turn is not None:
            self._turn(turn)
        self._agree_fully("after the opening turn")

        for _ in range(limit):
            event = self._drive()
            clock += run_ms_per_node
            m.tick(clock)
            self.scanner.poll()

            if event == "lost":
                self.log.append("LOST the line past {}".format(self.node))
                return "lost"

            if event == "bay":
                # The mission is still at the corridor junction: a bay
                # produces no junction event.
                # A bay produces no junction event, so at this moment the
                # mission is deliberately still at the corridor junction.
                # That is the lag at_bay() exists to close.
                if self.state.current_node != corridor_node(self.node):
                    self.desync.append(
                        "reached bay {} but mission was at {}, not the "
                        "corridor {}".format(self.node,
                                             self.state.current_node,
                                             corridor_node(self.node)))
                job = m.at_bay()
                self._agree_fully("in bay {}".format(self.node))
                if job == mis.PICK_UP:
                    pole = self.poles.get(self.node)
                    self.log.append("{}: collect {}".format(
                        self.node, pole or "nothing"))
                    m.collected(pole)
                elif job == mis.PUT_DOWN:
                    self.log.append("{}: deliver {}".format(
                        self.node, m.carrying))
                    self.occupied.add(self.node)
                    m.dropped()
                else:
                    self.log.append("{}: unexpected dead end ({})".format(
                        self.node, m.phase))
                    return "stuck"

                # main() reverses out: back at the corridor junction, still
                # facing into the bay.
                self.node = corridor_node(self.node)
                turn = m.leaving_bay()
                if turn is None:
                    return "done"
                self._turn(turn)
                self._agree_fully("backed out to {}".format(self.node))
                continue

            # A junction is only a position update now.
            turn = m.at_junction()
            self._agree_node("junction at {}".format(self.node))
            if turn is None:
                self.log.append("{}: run over ({})".format(self.node,
                                                           m.phase))
                return "done"

            # Standing beside a bay, the side sensors get their say before
            # the AGV drives on. On the robot this is asked every pass; one
            # look per bay decides the same thing here.
            self._present_bays()
            self.scanner.poll()
            turn_in = m.bay_beside()
            if turn_in is not None:
                self.log.append("{}: sensor says {} is free".format(
                    self.node, m.target))
                turn = turn_in

            self._turn(turn)
            self._agree_fully("turned {} at {}".format(turn, self.node))

        return "limit"


def main():
    print("which nodes produce a junction event")
    check("HOME has a marking heading N", has_marking("HOME", "N"), True)
    check("UU is just line, heading N", has_marking("UU", "N"), False)
    check("RH is just line, heading E", has_marking("RH", "E"), False)
    check("RC is a corner, so it has one", has_marking("RC", "E"), True)
    check("U3 has bays both sides", has_marking("U3", "N"), True)
    check("a bay has nothing to its sides",
          has_marking("LU3_BAY", "W"), False)
    def arrival_headings(node):
        """The headings the AGV can actually reach `node` on.

        You arrive facing h only if the node has an exit behind you, so a
        heading with no exit opposite it is not a direction the AGV can be
        travelling when it gets there.
        """
        return [h for h in COMPASS
                if JUNCTIONS[node][COMPASS[(COMPASS.index(h) + 2) % 4]]
                is not None]

    markless = sorted(n for n in JUNCTIONS
                      if not is_dead_end(n)
                      and not any(has_marking(n, h)
                                  for h in arrival_headings(n)))
    # LH and RH are the mirror of each other: both are a straight run with
    # no branch to either side, so neither produces an event.
    check("only UU, LH and RH are driven over unseen", markless,
          ["LH", "RH", "UU"])

    print("\nthe side a bay sits on, from the map alone")
    for pole, side, want in ((NORTH, dest.LEFT, "left"),
                             (NORTH, dest.RIGHT, "right"),
                             (SOUTH, dest.LEFT, "right"),
                             (SOUTH, dest.RIGHT, "left")):
        shelf = dest.shelf_for(pole, side)
        s = State(shelf[0][0], "N")
        check("{:5s} from the {:5s} scans the {}".format(pole, side, want),
              s.turn_to(shelf[0][1]), want)

    print("\none package: collect right, north pole, nothing on the racks")
    t = Table(poles={"R1_BAY": NORTH}, pickups=["R1_BAY"])
    check("run ends cleanly", t.run(), "done")
    check("no desync between table and mission", t.desync, [])
    check("one delivered", t.mission.delivered, 1)
    check("nothing still carried", t.mission.carrying, None)
    check("finished at home", t.node, "HOME")
    check("delivered to the first bay on rack B top",
          [l for l in t.log if "deliver" in l], ["RU1_BAY: deliver north"])

    print("\nthe first three bays are already full")
    t = Table(poles={"R1_BAY": NORTH}, pickups=["R1_BAY"],
              occupied=["RU1_BAY", "RU2_BAY", "RU3_BAY"])
    check("run ends cleanly", t.run(), "done")
    check("no desync", t.desync, [])
    check("skipped the three full ones", t.mission.skipped,
          ["RU1_BAY", "RU2_BAY", "RU3_BAY"])
    check("delivered to the fourth", [l for l in t.log if "deliver" in l],
          ["RU4_BAY: deliver north"])

    print("\nthe whole shelf is full")
    t = Table(poles={"R1_BAY": NORTH}, pickups=["R1_BAY"],
              occupied=["RU%d_BAY" % n for n in range(1, 7)])
    check("run ends cleanly", t.run(), "done")
    check("no desync", t.desync, [])
    check("nothing delivered", t.mission.delivered, 0)
    check("still carrying it", t.mission.carrying, NORTH)
    check("came home anyway", t.node, "HOME")

    print("\nan unreadable bay is skipped by default")
    t = Table(poles={"R1_BAY": NORTH}, pickups=["R1_BAY"],
              unreadable=["RU1_BAY"])
    t.run()
    check("skipped the one it could not read", t.mission.skipped,
          ["RU1_BAY"])
    check("delivered to the next one", [l for l in t.log if "deliver" in l],
          ["RU2_BAY: deliver north"])

    print("\nunknown_is_occupied=False tries it instead")
    t = Table(poles={"R1_BAY": NORTH}, pickups=["R1_BAY"],
              unreadable=["RU1_BAY"], unknown_is_occupied=False)
    t.run()
    check("nothing skipped", t.mission.skipped, [])
    check("delivered to the unreadable bay",
          [l for l in t.log if "deliver" in l], ["RU1_BAY: deliver north"])

    print("\nan empty loading bay moves on to the next pickup")
    t = Table(poles={"R2_BAY": SOUTH}, pickups=["R1_BAY", "R2_BAY"])
    check("run ends cleanly", t.run(), "done")
    check("no desync", t.desync, [])
    check("collected only the one that was there",
          t.mission.collected_count, 1)
    check("and delivered it", [l for l in t.log if "deliver" in l],
          ["RB1_BAY: deliver south"])

    print("\nall four loading bays, one of each pole per side")
    t = Table(poles={"R1_BAY": NORTH, "R2_BAY": SOUTH,
                     "L1_BAY": NORTH, "L2_BAY": SOUTH},
              pickups=["R1_BAY", "R2_BAY", "L1_BAY", "L2_BAY"])
    check("run ends cleanly", t.run(limit=800), "done")
    check("no desync", t.desync, [])
    check("four delivered", t.mission.delivered, 4)
    check("each to the right shelf", [l for l in t.log if "deliver" in l],
          ["RU1_BAY: deliver north", "RB1_BAY: deliver south",
           "LU1_BAY: deliver north", "LB1_BAY: deliver south"])
    check("home at the end", t.node, "HOME")

    print("\nrunning out of time cuts the run short")
    # A deliberately slow robot: 10 s of robot time per junction-to-junction
    # move, so the five minute limit bites part way through the four
    # packages. The point of the check is that it stops starting new ones
    # and still gets home.
    t = Table(poles={"R1_BAY": NORTH, "R2_BAY": SOUTH,
                     "L1_BAY": NORTH, "L2_BAY": SOUTH},
              pickups=["R1_BAY", "R2_BAY", "L1_BAY", "L2_BAY"],
              run_limit_ms=5 * 60 * 1000, cycle_reserve_ms=90 * 1000)
    check("run ends cleanly", t.run(limit=800, run_ms_per_node=10000), "done")
    check("no desync", t.desync, [])
    ok("gave up some pickups", t.mission.delivered < 4)
    ok("but delivered what it could", t.mission.delivered >= 1)
    check("still came home", t.node, "HOME")
    print("      ({} delivered, {} nodes, {:.0f} s of robot time)".format(
        t.mission.delivered, t.distance, t.mission.elapsed_ms / 1000))

    print("\nevery shelf is reachable from every loading bay")
    bad = []
    for bay, pole in [(b, p) for b in sorted(dest.LOADING_BAYS)
                      for p in (NORTH, SOUTH)]:
        t = Table(poles={bay: pole}, pickups=[bay])
        result = t.run(limit=600)
        if result != "done" or t.mission.delivered != 1 or t.desync:
            bad.append("{} {} -> {} {}".format(bay, pole, result, t.desync))
    check("all eight loading bay and pole combinations", bad, [])

    print("\nlosing the line on the way to a bay is not mistaken for arrival")
    t = Table(poles={"R1_BAY": NORTH}, pickups=["R1_BAY"])
    t.mission.start(now_ms=0)
    check("bound for R1_BAY but still at HOME",
          t.mission.expecting_bay, False)
    s = State("RC", "N")
    m = Mission(s, FakeScanner(), pickups=["R1_BAY"])
    check("at RC, one junction short of the spur",
          m.start(now_ms=0), "straight")
    check("not expecting a bay yet", m.expecting_bay, False)
    # Drive on to R1 and turn in, as the real loop would.
    m.at_junction()
    check("now at R1", s.current_node, "R1")
    check("facing into the spur", s.current_direction, "W")
    check("turned in, so losing the line means arrival",
          m.expecting_bay, True)

    print("\nreversing out of a bay is checked against the map")
    s = State("U1", "N")
    s.face("LU1_BAY")
    s.arrive("LU1_BAY")
    s.reverse_to("U1")
    check("back at the corridor", s.current_node, "U1")
    check("still facing into the bay", s.current_direction, "W")
    raised = False
    try:
        s.reverse_to("U2")
    except ValueError:
        raised = True
    check("reversing somewhere impossible raises", raised, True)
    raised = False
    try:
        State("HOME", "N").arrive("RH")
    except ValueError:
        raised = True
    check("arriving somewhere not in front raises", raised, True)

    print("\nthe planner reads no pins and keeps no stale route")
    t = Table(poles={"R1_BAY": NORTH}, pickups=["R1_BAY"])
    t.run()
    check("no sequence was ever set", t.state.sequence, ())
    source = open(os.path.join(REPO, "sw", "nav", "mission.py")).read()
    ok("mission.py touches no hardware", "machine" not in source)
    ok("and drives no motors", "drive" not in source.lower().split("driving")[0]
       or "self._drive" not in source)

    print()
    if failures:
        print("{} FAILED".format(len(failures)))
        for f in failures:
            print("  -", f)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
