"""Host-side tests for the QR reader.

    python3 tools/test_qr_parse.py

Runs sw/drivers/qr.py against the picostub clock, so the parser and the
confirmation logic are checked without a robot. Every accepted string here
is one the department's generator can actually produce; every rejected one
is a way a read can go wrong on the table.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "picostub"))
sys.path.insert(0, os.path.join(REPO, "sw"))

import utime                                     # noqa: E402
from drivers.qr import QRReader, Target, parse   # noqa: E402

failures = []


def check(what, got, want):
    ok = got == want
    if not ok:
        failures.append(what)
    print("  {}  {:<42} got {!r}".format("ok  " if ok else "FAIL", what, got))


def generated_codes():
    """Exactly what support/2025_mich_1_qr_codes.py writes."""
    for rack in ("A", "B"):
        for level in ("Upper", "Lower"):
            for bay in range(1, 7):
                yield "Rack {}, {}, {}".format(rack, level, bay), rack, level, bay


class FakeReader:
    """Stands in for TinyCodeReader. Returns whatever is queued."""

    def __init__(self):
        self.text = None
        self.fail = False

    def poll(self):
        if self.fail:
            raise OSError("I2C bus error")
        return self.text


def main():
    print("every code the department's generator produces")
    bad = 0
    for text, rack, level, bay in generated_codes():
        t = parse(text)
        if t is None or (t.rack, t.level, t.bay) != (rack, level, bay):
            bad += 1
    check("all 24 generated codes parse correctly", bad, 0)

    print("\ntolerated variations")
    check("trailing newline", parse("Rack A, Upper, 3\n"), Target("A", "Upper", 3))
    check("no space after commas", parse("Rack B,Lower,6"), Target("B", "Lower", 6))
    check("lower case", parse("rack a, upper, 1"), Target("A", "Upper", 1))

    print("\nrejected - these must never reach the route")
    for text, why in [
        (None, "nothing in view"),
        ("", "empty string"),
        ("Rack C, Upper, 3", "rack that does not exist"),
        ("Rack A, Middle, 3", "level that does not exist"),
        ("Rack A, Upper, 0", "bay below range"),
        ("Rack A, Upper, 7", "bay above range"),
        ("Rack A, Upper, two", "bay not a number"),
        ("Rack A, Upper", "truncated decode"),
        ("Rack A, Upper, 3, 4", "too many fields"),
        ("https://a,b,c", "a different QR code entirely"),
        ("Rck A, Upper, 3", "smudged keyword"),
    ]:
        check("reject: {}".format(why), parse(text), None)

    print("\nconfirmation - a target is reported once, after 3 identical reads")
    fake = FakeReader()
    reader = QRReader(fake, poll_ms=50, confirm_reads=3)

    def poll_once():
        utime.advance(50)
        return reader.poll()

    fake.text = "Rack A, Upper, 3"
    check("read 1 of 3", poll_once(), None)
    check("read 2 of 3", poll_once(), None)
    check("read 3 of 3 reports", poll_once(), Target("A", "Upper", 3))
    check("stays quiet afterwards", poll_once(), None)
    check("still quiet", poll_once(), None)

    print("\nrate limit")
    reader.reset()
    fake.text = "Rack B, Lower, 2"
    utime.advance(50)
    reader.poll()
    before = reader.confirmations
    reader.poll()           # same millisecond - must not count
    check("second call inside the window is ignored", reader.confirmations, before)

    print("\na changed code restarts the count")
    reader.reset()
    fake.text = "Rack A, Upper, 1"
    poll_once()
    poll_once()
    fake.text = "Rack B, Lower, 5"
    poll_once()
    check("count restarted on the new code", reader.confirmations, 1)
    check("old code not reported", reader.reading, Target("B", "Lower", 5))

    print("\nan interrupted read cannot be completed by another package")
    reader.reset()
    fake.text = "Rack A, Upper, 4"
    poll_once()
    poll_once()
    fake.text = None        # code leaves the field of view
    poll_once()
    fake.text = "Rack A, Upper, 4"
    check("back to 1 after the gap", (poll_once(), reader.confirmations), (None, 1))

    print("\nI2C failure is survivable")
    reader.reset()
    fake.fail = True
    check("poll returns None, does not raise", poll_once(), None)
    poll_once()
    check("errors are counted, not hidden", reader.error_count, 2)
    fake.fail = False
    fake.text = "Rack B, Upper, 6"
    poll_once()
    check("error count clears on a good read", reader.error_count, 0)

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
