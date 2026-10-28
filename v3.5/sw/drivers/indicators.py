"""Status lamps.

M1 requires a flashing amber beacon whenever the AGV is in operation, at a
rate the human eye reads as a hazard warning.  The lamp must stop flashing
when the AGV stops, which proves to the judges that it is actually being
switched rather than wired permanently on.

LEDs are driven through the opto-isolator / transistor stage on the interface
board, never directly from a Pico pin (M1 build spec).

update() must be called once per main-loop pass.  It never blocks: it only
compares timestamps and toggles the output when the interval has elapsed.
Flashing with sleep() instead would stall the stop-button check for the whole
off period, which is not acceptable on a safety control.
"""

from machine import Pin
from utime import ticks_ms, ticks_diff

from drivers.magnet import NORTH, SOUTH


class FlashingLamp:
    def __init__(self, pin_no, period_ms):
        self._pin = Pin(pin_no, Pin.OUT)
        self._period_ms = period_ms
        self._enabled = False
        self._state = 0
        self._last_toggle_ms = ticks_ms()
        self._pin.value(0)

    def on(self):
        """Start flashing."""
        if not self._enabled:
            self._enabled = True
            # Light immediately so the beacon is visibly on from the moment
            # the AGV starts to move.
            self._state = 1
            self._pin.value(1)
            self._last_toggle_ms = ticks_ms()

    def off(self):
        """Stop flashing and leave the lamp dark."""
        self._enabled = False
        self._state = 0
        self._pin.value(0)

    def update(self):
        """Toggle the lamp if the flash interval has elapsed."""
        if not self._enabled:
            return
        if ticks_diff(ticks_ms(), self._last_toggle_ms) >= self._period_ms:
            self._state ^= 1
            self._pin.value(self._state)
            self._last_toggle_ms = ticks_ms()


class PoleIndicator:
    """Steady red for a north-pole package, steady green for south (M1 spec)."""

    def __init__(self, red_pin, green_pin):
        self._red = Pin(red_pin, Pin.OUT)
        self._green = Pin(green_pin, Pin.OUT)
        self.off()

    def show(self, pole):
        """Light the lamp for pole, or neither for None. Safe to repeat."""
        self._red.value(1 if pole == NORTH else 0)
        self._green.value(1 if pole == SOUTH else 0)

    def off(self):
        self.show(None)
