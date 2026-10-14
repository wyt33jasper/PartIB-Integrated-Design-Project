"""Host-side stand-in for MicroPython's utime module.

Uses a virtual clock so a simulation can run faster than real time.
"""

_now_ms = 0


def advance(ms):
    global _now_ms
    _now_ms += ms


def ticks_ms():
    return _now_ms


def ticks_diff(a, b):
    return a - b


def sleep_ms(ms):
    advance(ms)
