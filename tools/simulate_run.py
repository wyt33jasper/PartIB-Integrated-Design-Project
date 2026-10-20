"""Boot sw/main.py on a desktop and drive it through a whole run.

    python3 tools/simulate_run.py

Nothing in sw/ is stubbed except machine and utime, which the Pico provides
for real. main.main() is called exactly as the Pico calls it, so this
exercises the real integration - the pin allocation, the drivers, the
junction detector, the mission planner and the turn phases - rather than any
of it in isolation.

utime.sleep_ms is replaced before any sw module is imported, because every
one of them binds sleep_ms at import time; patching afterwards would leave
them calling the original and the virtual clock would never advance.

Each tick the world decides what the sensors see:

  * counter-rotating motors mean the AGV is mid-turn, so the inner trackers
    lose the line and then find it again, which is what _perform_turn waits
    for
  * driving forward carries the AGV along the line, one map node every
    NODE_TICKS; a node with a marking to one side shows it to the outer
    trackers, and a bay shows the inner pair black because the line has
    run out
  * reversing carries it back out of a bay
  * at a loading bay holding a package, the Hall pin for that pole goes
    high, so the real DigitalHallPair and MagnetSensor do the identifying
  * the side rangefinder pins report whichever bays the world has decided
    are already full

It also reports the pin assignment as main() actually built it, which is how
a pin claimed twice gets caught.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))
sys.path.insert(0, os.path.join(REPO, "sw"))

import utime                                            # noqa: E402
from machine import Pin                                 # noqa: E402

# Main-loop passes per junction-to-junction move. This has to be large
# compared with TURN_DRIVE_OVER_MS, or the AGV would cross whole map nodes
# during the drive-over phase of a single turn - 1200 ms of driving has to
# be a fraction of the gap between junctions, not several of them.
NODE_TICKS = int(os.environ.get("NODE_TICKS", 500))  # 2.5 s per move
MARKING_TICKS = 8           # how long a marking stays under the trackers
OFF_LINE_TICKS = 30         # how long the inner pair is off the line in a turn
MAX_TICKS = 400000

CLAIMS = {}


def install_pin_tracking(config):
    """Record every Pin construction, so a double claim is visible."""
    names = {}
    for attr in dir(config):
        value = getattr(config, attr)
        if attr.endswith("_PIN") and isinstance(value, int):
            names[value] = attr
    for side, pins in getattr(config, "RANGER_PINS", {}).items():
        names.setdefault(pins["pin"], "RANGER_{}".format(side.upper()))

    original = Pin.__init__

    def tracked(self, no, mode=None, pull=None):
        original(self, no, mode, pull)
        CLAIMS.setdefault(no, []).append(names.get(no, "GP%d" % no))

    Pin.__init__ = tracked


def motor_speed(motor):
    """Signed speed of one motor, read back off its own pins."""
    if motor._pwm.duty == 0:
        return 0
    forward = motor._dir.value() == 0
    if motor._reverse:
        forward = not forward
    return motor._pwm.duty if forward else -motor._pwm.duty


class World:
    """The table, as far as the AGV's sensors can tell."""

    def __init__(self, config, occupied=(), poles=None):
        self.config = config
        self.node = config.INITIAL_NODE
        self.occupied = set(occupied)
        self.poles = dict(poles or {})
        self.robot = None
        self.ticks = 0
        self.node_ticks = 0
        self.marking_ticks = 0
        self.turn_ticks = 0
        self.events = []
        self.lost = False

        from nav.map import COMPASS, JUNCTIONS, ahead, corridor_node, \
            has_marking, is_dead_end
        self._COMPASS = COMPASS
        self._JUNCTIONS = JUNCTIONS
        self._ahead = ahead
        self._corridor_node = corridor_node
        self._has_marking = has_marking
        self._is_dead_end = is_dead_end

    # -------------------------------------------------------------- sensors

    @property
    def heading(self):
        """The heading the AGV was last told to face."""
        return self.robot.mission.state.current_direction

    def _set(self, pin_no, value):
        pin = Pin.registry.get(pin_no)
        if pin is not None:
            pin.value(1 if value else 0)

    def _show(self, line_l, line_r, turn_l, turn_r):
        c = self.config
        self._set(c.LINE_TRACKER_L_PIN, line_l)
        self._set(c.LINE_TRACKER_R_PIN, line_r)
        self._set(c.TURN_TRACKER_L_PIN, turn_l)
        self._set(c.TURN_TRACKER_R_PIN, turn_r)

    def _show_magnet(self):
        """High on the pin for whichever pole is in front of the gripper."""
        c = self.config
        pole = self.poles.get(self.node) if self._is_dead_end(self.node) \
            else None
        active = 1 if c.MAGNET_ACTIVE_HIGH else 0
        self._set(c.MAGNET_NORTH_PIN, active if pole == "north" else 1 - active)
        self._set(c.MAGNET_SOUTH_PIN, active if pole == "south" else 1 - active)

    def _show_rangers(self):
        """Report whichever bays sit beside the AGV right now."""
        c = self.config
        if c.RANGER_MODE != "digital":
            return
        index = self._COMPASS.index(self.heading)
        sides = (("left", self._COMPASS[(index - 1) % 4]),
                 ("right", self._COMPASS[(index + 1) % 4]))
        for side, compass in sides:
            neighbour = self._JUNCTIONS[self.node][compass]
            near = (neighbour is not None
                    and self._is_dead_end(neighbour)
                    and neighbour in self.occupied)
            active = 1 if c.RANGER_ACTIVE_HIGH else 0
            self._set(c.RANGER_PINS[side]["pin"],
                      active if near else 1 - active)

    # ----------------------------------------------------------------- tick

    def tick(self, ms):
        self.ticks += 1
        if self.ticks > MAX_TICKS:
            raise KeyboardInterrupt("tick limit reached")

        robot = self.robot
        if robot is None:
            return

        # Press the start button a few passes in.
        if self.ticks == 5:
            self.events.append("start button pressed")
            Pin.registry[self.config.BUTTON_PIN].press()
            return

        self._show_magnet()
        self._show_rangers()

        left = motor_speed(robot.drive._left)
        right = motor_speed(robot.drive._right)

        if left * right < 0:                        # counter-rotating: a turn
            self.turn_ticks += 1
            self.node_ticks = 0
            on_line = self.turn_ticks > OFF_LINE_TICKS
            self._show(on_line, on_line, 0, 0)
            return

        self.turn_ticks = 0

        if left < 0 and right < 0:                  # reversing out of a bay
            self.node_ticks += 1
            if self.node_ticks >= NODE_TICKS and self._is_dead_end(self.node):
                self.node = self._corridor_node(self.node)
                self.node_ticks = 0
                self.events.append("backed out to {}".format(self.node))
            self._show(1, 1, 0, 0)
            return

        if left == 0 and right == 0:                # stopped
            self.node_ticks = 0
            if self._is_dead_end(self.node):
                self._show(0, 0, 0, 0)              # still in the bay
            else:
                self._show(1, 1, 0, 0)
            return

        # Driving forward along the line.
        if self._is_dead_end(self.node):
            self._show(0, 0, 0, 0)                  # the line has run out
            return

        self.node_ticks += 1
        if self.node_ticks >= NODE_TICKS:
            self.node_ticks = 0
            nxt = self._ahead(self.node, self.heading)
            if nxt is None:
                self.lost = True
                self.events.append(
                    "drove off the end of the line past {}".format(self.node))
                self._show(0, 0, 0, 0)
                return
            self.node = nxt
            if self._is_dead_end(nxt):
                self.events.append("line ran out in {}".format(nxt))
            elif self._has_marking(nxt, self.heading):
                self.marking_ticks = MARKING_TICKS
                self.events.append("marking at {}".format(nxt))

        if self._is_dead_end(self.node):
            self._show(0, 0, 0, 0)
            return

        if self.marking_ticks > 0:
            self.marking_ticks -= 1
            self._show(1, 1, 1, 1)
        else:
            self._show(1, 1, 0, 0)


def main():
    import config

    # What this run is: sensors fitted, a package on each loading bay, and
    # the first two bays of rack B top already full.
    config.RANGER_MODE = "digital"
    config.GRIPPER_FITTED = True
    config.GRIPPER_CLOSE_MS = 800
    config.GRIPPER_OPEN_MS = 800

    install_pin_tracking(config)

    world = World(config,
                  occupied=["RU1_BAY", "RU2_BAY"],
                  poles={"R1_BAY": "north", "R2_BAY": "south",
                         "L1_BAY": "north", "L2_BAY": "south"})

    real_sleep = utime.sleep_ms

    def world_sleep_ms(ms):
        real_sleep(ms)
        world.tick(ms)

    # Must happen before any sw module imports sleep_ms from utime.
    utime.sleep_ms = world_sleep_ms

    import main as app

    original_build = app.build

    def build():
        robot = original_build()
        world.robot = robot
        return robot
    app.build = build

    print("start: {} facing {}".format(config.INITIAL_NODE,
                                       config.INITIAL_DIRECTION))
    print("pickups: {}".format(", ".join(config.PICKUP_ORDER)))
    print("already full: {}".format(", ".join(sorted(world.occupied))))
    print("booting sw/main.py")
    print("-" * 68)

    try:
        app.main()
    except KeyboardInterrupt as exc:
        print("[world] stopped: {}".format(exc))
    except Exception as exc:
        print("[world] main() raised {}: {}".format(type(exc).__name__, exc))
        import traceback
        traceback.print_exc()

    print("-" * 68)
    mission = world.robot.mission
    print("delivered {} package(s); skipped {}".format(
        mission.delivered, mission.skipped or "nothing"))
    print("ended at {} (world says {}), phase {}".format(
        mission.state.current_node, world.node, mission.phase))
    print("{} ticks = {:.0f} s of robot time ({:.1f} s per junction-to-"
          "junction move, limit {:.0f} s)".format(
              world.ticks, world.ticks * config.LOOP_PERIOD_MS / 1000,
              NODE_TICKS * config.LOOP_PERIOD_MS / 1000,
              config.RUN_LIMIT_MS / 1000))
    print("world agrees with the mission: {}".format(
        world.node == mission.state.current_node))

    print("\nwhat the world did")
    for event in world.events:
        print("  " + event)

    print("\npin assignment as main() actually built it")
    for no in sorted(CLAIMS):
        who = CLAIMS[no]
        flag = "   <-- CLAIMED TWICE" if len(set(who)) > 1 else ""
        print("  GP{:<3} {}{}".format(no, ", ".join(sorted(set(who))), flag))

    doubled = [no for no, who in CLAIMS.items() if len(set(who)) > 1]
    return 1 if (doubled or world.lost) else 0


if __name__ == "__main__":
    sys.exit(main())
