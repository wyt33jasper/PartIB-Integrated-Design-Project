"""Host tests for the bay detector.

    python3 tools/test_bay.py

Covers the two photographed scenarios: a box in the first bay (drive on,
take the second) and an empty first bay (turn in straight away).
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))
sys.path.insert(0, os.path.join(REPO, "sw"))

import utime                                      # noqa: E402
import config                                     # noqa: E402
from nav.bay import BayDetector, EMPTY, FULL, LEFT, NONE, RIGHT  # noqa: E402

failures = []


def check(what, got, want):
    ok = got == want
    if not ok:
        failures.append("{} (got {!r}, wanted {!r})".format(what, got, want))
    print("  {}  {:<48} {!r}".format("ok  " if ok else "FAIL", what, got))


class FakeSensor:
    """Reads back whatever mm is set on it. None means no new measurement."""

    def __init__(self, mm=None):
        self.mm = mm
        self.last_mm = mm
        self.reads = 0

    def read(self):
        self.reads += 1
        self.last_mm = self.mm
        return self.mm


def detector(left_mm=None, right_mm=None):
    left, right = FakeSensor(left_mm), FakeSensor(right_mm)
    bay = BayDetector(left, right,
                      full_mm=config.BAY_FULL_MM,
                      confirm_reads=config.BAY_CONFIRM_READS,
                      timeout_ms=config.BAY_TIMEOUT_MS)
    return bay, left, right


def run_until_result(bay, passes=20, step_ms=10):
    """Poll as the main loop would, until an event or we run out of passes."""
    for _ in range(passes):
        result = bay.poll()
        if result != NONE:
            return result
        utime.advance(step_ms)
    return NONE


def main():
    print("nothing happens until a junction starts a measurement")
    bay, left, _ = detector(left_mm=900)
    check("idle poll reports nothing", bay.poll(), NONE)
    check("not measuring", bay.measuring, False)
    check("sensor was never read", left.reads, 0)

    print("\nempty bay: far readings confirm it")
    bay, left, _ = detector(left_mm=900)
    bay.start(LEFT)
    check("now measuring on the left", bay.side, LEFT)
    for i in range(config.BAY_CONFIRM_READS - 1):
        check("read {} of {} - not decided yet".format(
            i + 1, config.BAY_CONFIRM_READS), bay.poll(), NONE)
    check("the last read decides it", bay.poll(), EMPTY)
    check("bay_is_empty() agrees", bay.bay_is_empty(), True)
    check("measurement is over", bay.measuring, False)
    check("one bay counted", bay.count, 1)

    print("\nfull bay: one near reading is enough")
    bay, left, _ = detector(left_mm=60)
    bay.start(LEFT)
    check("decided on the first read", bay.poll(), FULL)
    check("bay_is_empty() is False", bay.bay_is_empty(), False)
    check("and it stopped reading", bay.measuring, False)

    print("\na box part way through still means full")
    bay, left, _ = detector(left_mm=900)
    bay.start(LEFT)
    bay.poll()
    left.mm = 60
    check("the near reading wins", bay.poll(), FULL)

    print("\nexactly on the threshold counts as full")
    bay, left, _ = detector(left_mm=config.BAY_FULL_MM)
    bay.start(LEFT)
    check("at the threshold", bay.poll(), FULL)
    bay, left, _ = detector(left_mm=config.BAY_FULL_MM + 1)
    bay.start(LEFT)
    check("one mm past it is not", run_until_result(bay), EMPTY)

    print("\nthe two sides are independent")
    bay, left, right = detector(left_mm=900, right_mm=60)
    bay.start(RIGHT)
    check("right side reads the box", bay.poll(), FULL)
    check("left sensor untouched", left.reads, 0)
    bay.start(LEFT)
    check("left side reads empty", run_until_result(bay), EMPTY)

    print("\nno readings in time means full, not empty")
    bay, left, _ = detector(left_mm=None)      # sensor never returns a value
    bay.start(LEFT)
    utime.advance(config.BAY_TIMEOUT_MS)
    check("timeout is fail-safe", bay.poll(), FULL)
    check("bay_is_empty() is False", bay.bay_is_empty(), False)

    print("\nphoto 1: box in the first bay, second bay free")
    bay, left, _ = detector()
    left.mm = 60                                # first junction: box
    bay.start(LEFT)
    check("first bay is full - drive on", run_until_result(bay), FULL)
    left.mm = 900                               # second junction: clear
    bay.start(LEFT)
    check("second bay is empty - turn in", run_until_result(bay), EMPTY)
    check("two bays measured", bay.count, 2)

    print("\nphoto 2: first bay already free")
    bay, left, _ = detector(left_mm=900)
    bay.start(LEFT)
    check("turn into the first bay", run_until_result(bay), EMPTY)
    check("only one bay measured", bay.count, 1)

    print("\na turn cancels any measurement in progress")
    bay, left, _ = detector(left_mm=900)
    bay.start(LEFT)
    bay.poll()
    bay.cancel()
    check("not measuring", bay.measuring, False)
    check("and poll stays quiet", bay.poll(), NONE)

    print("\nbad input is rejected")
    raised = False
    try:
        bay.start("sideways")
    except ValueError:
        raised = True
    check("unknown side raises", raised, True)

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
