"""Host stand-in for MicroPython's micropython module."""


def const(value):
    """On the Pico this is a compiler hint; on the host it is the identity."""
    return value
