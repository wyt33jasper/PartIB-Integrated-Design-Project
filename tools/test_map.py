"""Check nav/map.py describes a table the AGV can actually work.

    python3 tools/test_map.py

The map is sixty-odd nodes typed in by hand, and a single wrong compass
letter is invisible by inspection but makes one route impossible. Since
every destination is now planned from the map rather than typed out, a
mistake in it shows up as the AGV driving somewhere unaccountable on the
table. These checks find it in a second instead.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))
sys.path.insert(0, os.path.join(REPO, "sw"))

from nav import destination as dest                    # noqa: E402
from drivers.magnet import NORTH, SOUTH                # noqa: E402
from nav.map import (COMPASS, JUNCTIONS, corridor_node,  # noqa: E402
                     exits, is_dead_end, path_find)

failures = []


def check(what, got, want):
    ok = got == want
    if not ok:
        failures.append("{} (got {!r}, wanted {!r})".format(what, got, want))
    print("  {}  {:<52} {!r}".format("ok  " if ok else "FAIL", what, got))


def main():
    print("the map is well formed")
    check("every node lists all four compass directions",
          sorted({d for n in JUNCTIONS for d in JUNCTIONS[n]}),
          sorted(COMPASS))
    unknown = sorted({t for n in JUNCTIONS for t in JUNCTIONS[n].values()
                      if t is not None and t not in JUNCTIONS})
    check("every exit leads to a node that exists", unknown, [])

    print("\nevery connection is two-way, and agrees on the direction")
    opposite = {"N": "S", "S": "N", "E": "W", "W": "E"}
    wrong = []
    for node in JUNCTIONS:
        for direction, target in JUNCTIONS[node].items():
            if target is None:
                continue
            back = JUNCTIONS[target][opposite[direction]]
            if back != node:
                wrong.append("{} -{}-> {}, but {} -{}-> {}".format(
                    node, direction, target, target, opposite[direction],
                    back))
    check("no one-way or mismatched links", wrong, [])

    print("\nbays")
    bays = sorted(n for n in JUNCTIONS if is_dead_end(n))
    check("28 dead ends - 24 storage and 4 loading", len(bays), 28)
    check("all of them are named _BAY",
          [n for n in bays if not n.endswith("_BAY")], [])
    check("and nothing named _BAY is anything else",
          sorted(n for n in JUNCTIONS
                 if n.endswith("_BAY") and not is_dead_end(n)), [])
    check("each has exactly one exit",
          sorted({len(exits(b)) for b in bays}), [1])

    print("\nthe destination rule covers every bay on the map")
    storage = set()
    for rack in (dest.RACK_A, dest.RACK_B):
        for level in (dest.TOP, dest.BOTTOM):
            storage.update(dest.bay_nodes(rack, level))
    check("24 storage bays", len(storage), 24)
    check("no node is both storage and loading",
          sorted(storage & set(dest.LOADING_BAYS)), [])
    check("together they are every bay",
          sorted(set(bays) - storage - set(dest.LOADING_BAYS)), [])

    print("\neach shelf is one corridor, and its bays are all on one side")
    for pole, side in ((NORTH, dest.LEFT), (NORTH, dest.RIGHT),
                       (SOUTH, dest.LEFT), (SOUTH, dest.RIGHT)):
        shelf = dest.shelf_for(pole, side)
        corridors = [c for c, _ in shelf]
        # Consecutive corridor nodes must be directly connected, or the AGV
        # cannot drive from one bay to the next.
        gaps = [(a, b) for a, b in zip(corridors, corridors[1:])
                if b not in JUNCTIONS[a].values()]
        rack, level = dest.destination(pole, side)
        check("rack {} {:6s}: six bays off a connected run".format(
            rack, level), (len(shelf), gaps), (6, []))

    print("\nevery delivery route the mission can need")
    bad = []
    for start in sorted(dest.LOADING_BAYS):
        for bay in sorted(storage):
            # The route planned is loading bay -> shelf corridor, then along
            # it; and afterwards back home.
            for end in (corridor_node(bay), "HOME"):
                try:
                    path_find(start, end)
                except ValueError as exc:
                    bad.append("{} -> {}: {}".format(start, end, exc))
    check("all 96 loading-bay to shelf routes, and home again", bad, [])

    print("\nno bay is a short cut")
    # path_find must never route through a bay: driving into a dead end and
    # out again to get somewhere else would be nonsense, and would also fire
    # the bay-arrival branch in main().
    through = []
    for start in ("HOME", "U1", "RB3", "LB5", "R1_BAY"):
        for end in ("U6", "RB6", "LB6", "HOME", "L2_BAY"):
            if start == end:
                continue
            steps = path_find(start, end)
            for _, _, node in steps[:-1]:
                if is_dead_end(node):
                    through.append("{} -> {} goes through {}".format(
                        start, end, node))
    check("routes never pass through one", through, [])

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
