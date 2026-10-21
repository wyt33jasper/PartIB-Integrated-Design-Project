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


class SteadyLamp:
    """A lamp that is simply on or off.

    update() exists only so that every lamp has the same interface and the
    main loop can call it blindly; a steady lamp has nothing to update.
    """

    def __init__(self, pin_no):
        self._pin = Pin(pin_no, Pin.OUT)
        self._pin.value(0)

    def set(self, lit):
        self._pin.value(1 if lit else 0)

    def on(self):
        self.set(True)

    def off(self):
        self.set(False)

    def update(self):
        pass


class PoleIndicator:
    """Red for a north pole package, green for a south pole one.

    From the task spec: "It must clearly display a steady red light if the
    package has a magnetic field north pole, green light if south pole".

    Both lamps are off when the AGV is carrying nothing, which keeps the
    display honest - a lamp left lit from the previous package would tell
    the judges the AGV had identified something it has not.

    show() is idempotent, so the main loop can call it every pass with the
    magnet sensor's current reading without tracking what changed.
    """

    def __init__(self, red_pin, green_pin, north=NORTH, south=SOUTH):
        """north and south are the pole labels, not tuning values."""
        self._red = SteadyLamp(red_pin)
        self._green = SteadyLamp(green_pin)
        self._north = north
        self._south = south

    def show(self, pole):
        """Light the lamp matching the pole, or neither when pole is None."""
        self._red.set(pole == self._north)
        self._green.set(pole == self._south)

    def off(self):
        self._red.off()
        self._green.off()

    def update(self):
        pass
