"""Check the rear-sensor bay manoeuvre, on the host, against v3.5/sw.

    python3 tools/test_rear_bay.py

The sensor moved from the sides of the AGV to the back, so a bay is now
measured by turning away from it and reversing up to its mouth. That is a
sequence of open-loop moves around one sensor reading, and the two things
that can go wrong are both invisible by inspection:

  * a verdict reached on readings taken before the AGV arrives, which would
    call a full bay empty and stack a second package on the first;
  * finishing the manoeuvre off the line, which main.py cannot tell apart
    from the far end of a delivery bay - the box would be released in the
    corridor.

Both are checked here with a fake sensor and fake trackers, so the real
code paths in main._probe_bay() run. The mission walkthrough underneath
then drives the whole collect-deliver cycle topologically.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))
sys.path.insert(0, os.path.join(REPO, "v3.5", "sw"))

import config                                                   # noqa: E402
import main                                                     # noqa: E402
from drivers.distance import NoSensor                           # noqa: E402
from nav.bay import BayDetector, EMPTY, FULL, NONE              # noqa: E402
from nav.mission import COLLECT_BAYS, Mission                   # noqa: E402
from nav.navigator import State                                 # noqa: E402

failures = []


def check(what, got, want):
    if got == want:
        print("  ok   {}".format(what))
    else:
        print("  FAIL {} - got {!r}, want {!r}".format(what, got, want))
        failures.append(what)


# --------------------------------------------------------------- fake hardware
class Button:
    def __init__(self, press_after=None):
        self.polls = 0
        self._at = press_after

    def was_pressed(self):
        self.polls += 1
        return self._at is not None and self.polls >= self._at


class Beacon:
    def update(self):
        pass


class Drive:
    """Records every motor command so the manoeuvre can be read back."""

    def __init__(self):
        self.log = []

    def move(self, speed, steer=0):
        self.log.append(("move", speed))

    def rotate(self, direction, speed):
        self.log.append(("rotate", direction))

    def u_turn(self, speed):
        self.log.append(("u_turn",))

    def stop(self):
        self.log.append(("stop",))

    def rotations(self):
        return [c[1] for c in self.log if c[0] == "rotate"]


class Trackers:
    """on_line is returned in turn; a single value repeats for ever."""

    def __init__(self, on_line=(True,)):
        self._seq = list(on_line)

    def line_trackers(self):
        value = self._seq[0] if len(self._seq) == 1 else self._seq.pop(0)
        return (value, value)

    def turn_trackers(self):
        return (False, False)


class Sensor:
    """readings are mm or None, returned in turn; a single value repeats."""

    def __init__(self, readings):
        self._seq = list(readings)
        self.last_mm = None

    def read(self):
        mm = self._seq[0] if len(self._seq) == 1 else self._seq.pop(0)
        if mm is not None:
            self.last_mm = mm
        return mm


class Exploding:
    last_mm = None

    def read(self):
        raise AssertionError("the sensor was read while not measuring")


def detector_for(sensor):
    return BayDetector(sensor, config.BAY_FULL_MM, config.BAY_NEAR_READS,
                       config.BAY_CONFIRM_READS)


def probe(sensor, side="left", press_after=None, on_line=(True,)):
    """Run the real _probe_bay(). Returns (verdict, detector, drive)."""
    detector = detector_for(sensor)
    drive = Drive()
    verdict = main._probe_bay(Button(press_after), Beacon(), drive,
                              Trackers(on_line), detector, "test_BAY", side)
    return verdict, detector, drive


def first_verdict(detector, polls=8):
    for _ in range(polls):
        verdict = detector.poll()
        if verdict != NONE:
            return verdict
    return NONE


# ------------------------------------------------------- the manoeuvre itself
print("An empty bay reads far once the AGV has stopped at it")
verdict, detector, drive = probe(Sensor([400]))
check("the verdict is EMPTY", verdict, EMPTY)
check("bay_is_empty() agrees", detector.bay_is_empty(), True)
check("the reading is kept for the console", detector.last_mm, 400)
check("two rotations: away, then the 180 spin", len(drive.rotations()), 2)
check("a bay on the left is reached by turning right twice",
      drive.rotations(), [1, 1])
check("the approach reverses, and slowly",
      ("move", -config.BAY_REVERSE_SPEED) in drive.log, True)
check("the motors are stopped at the bay mouth",
      ("stop",) in drive.log, True)

print("A bay on the right is the mirror image")
verdict, detector, drive = probe(Sensor([400]), side="right")
check("the verdict is EMPTY", verdict, EMPTY)
check("both rotations go the other way", drive.rotations(), [-1, -1])

print("A package near the mouth ends the reverse early")
verdict, detector, drive = probe(Sensor([80]))
check("the verdict is FULL", verdict, FULL)
check("bay_is_empty() is False", detector.bay_is_empty(), False)
check("the AGV rotates back the way it came, not on round",
      drive.rotations(), [1, -1])

print("A package only visible from the mouth is still found")
verdict, detector, drive = probe(Sensor([900] * 15 + [90]))
check("the verdict is FULL", verdict, FULL)

print("A sensor that is not fitted reads every bay full")
verdict, detector, drive = probe(NoSensor())
check("the verdict is FULL, so the bay is skipped", verdict, FULL)
check("bay_is_empty() is False", detector.bay_is_empty(), False)

print("The stop button ends the manoeuvre wherever it is")
verdict, detector, drive = probe(Sensor([400]), press_after=3)
check("the verdict is None", verdict, None)
check("the motors are left stopped", drive.log[-1], ("stop",))

print("Losing the line after the spin refuses the delivery")
verdict, detector, drive = probe(Sensor([400]), on_line=[False] * 4000)
check("the verdict is None, so no box is dropped in the corridor",
      verdict, None)
check("the bay itself still read empty", detector.verdict, EMPTY)

# ------------------------------------------------------------- the thresholds
print("The threshold is inclusive, and takes the configured run of readings")
detector = detector_for(Sensor([config.BAY_FULL_MM]))
detector.start("x")
detector.settle()
check("exactly BAY_FULL_MM is a package", first_verdict(detector), FULL)

detector = detector_for(Sensor([config.BAY_FULL_MM + 1]))
detector.start("x")
detector.settle()
check("one mm further is empty", first_verdict(detector), EMPTY)

detector = detector_for(Sensor([80]))
detector.start("x")
check("FULL takes BAY_NEAR_READS readings",
      [detector.poll() for _ in range(config.BAY_NEAR_READS)][-1], FULL)

detector = detector_for(Sensor([900]))
detector.start("x")
detector.settle()
check("EMPTY takes BAY_CONFIRM_READS readings",
      [detector.poll() for _ in range(config.BAY_CONFIRM_READS)][-1], EMPTY)

print("Readings taken on the way in cannot call a bay empty")
detector = detector_for(Sensor([900]))
detector.start("x")
check("twenty far readings while reversing decide nothing",
      set(detector.poll() for _ in range(20)), set([NONE]))
detector.settle()
check("empty only once settle() has been called",
      first_verdict(detector), EMPTY)

print("An idle detector never touches the sensor")
detector = detector_for(Exploding())
check("poll() before start() is inert",
      [detector.poll() for _ in range(50)], [NONE] * 50)
detector.start("x")
detector.cancel()
check("poll() after cancel() is inert", detector.poll(), NONE)


# --------------------------------------------------------- mission walkthrough
def walk(world, steps=160):
    """Drive the mission topologically. world = {bay: a package is in it}.

    Returns (packages delivered, where it finished, the bays used).
    """
    state = State(config.INITIAL_NODE, config.INITIAL_DIRECTION,
                  [config.INITIAL_NODE])
    mission = Mission(state, config.FIRST_PICKUP, config.HOME_NODE,
                      config.RUN_LIMIT_MS, config.CYCLE_RESERVE_MS,
                      config.EMPTY_BAYS_BEFORE_HOME)
    mission.start()
    used = []
    for _ in range(steps):
        misses = state.previous_node() in main.MISS_CASE_DELIVERY_BAYS
        side, bay = mission.bay_here()

        if side is not None:
            if world.get(bay):                  # full - on down the shelf
                mission.skip_bay()
                if state.junction_turn() is None:
                    return mission.delivered_count, state.current_node, used
                continue
            mission.take_bay(bay)               # empty - deliver into it
            state.junction_turn()
            world[bay] = True
            used.append(bay)
            mission.delivered()
            state.junction_turn()               # the u-turn back out
            continue

        if state.junction_turn() is None:
            return mission.delivered_count, state.current_node, used
        if misses:
            state.junction_turn()
        if state.current_node in COLLECT_BAYS:
            pole = "south" if world.get(state.current_node) else None
            world[state.current_node] = False
            mission.collected(pole)
    raise AssertionError("the mission never finished")


print("A package in every pickup bay is delivered and the AGV comes home")
delivered, finished_at, used = walk(dict((b, True) for b in COLLECT_BAYS))
check("all four are delivered", delivered, 4)
check("the run ends at HOME", finished_at, config.HOME_NODE)
check("each one goes in its own bay", len(set(used)), 4)

print("A shelf that is full up to one bay still gets used")
world = {"R2_BAY": True}
for bay in ("RB1_BAY", "RB2_BAY", "RB3_BAY", "RB4_BAY", "RB6_BAY"):
    world[bay] = True
delivered, finished_at, used = walk(world)
check("the one free bay is found", used, ["RB5_BAY"])
check("the run ends at HOME", finished_at, config.HOME_NODE)

print("A shelf that is completely full falls back to the other deck")
world = {"R2_BAY": True}
for n in range(1, 7):
    world["RB%d_BAY" % n] = True
delivered, finished_at, used = walk(world)
check("the package goes to the upper deck instead", used, ["RU1_BAY"])
check("the run ends at HOME", finished_at, config.HOME_NODE)

print("Empty pickup bays send the AGV home rather than round for ever")
delivered, finished_at, used = walk({})
check("nothing is delivered", delivered, 0)
check("the run ends at HOME", finished_at, config.HOME_NODE)


print()
if failures:
    print("{} FAILED".format(len(failures)))
    for what in failures:
        print("  - {}".format(what))
    sys.exit(1)
print("all checks passed")
