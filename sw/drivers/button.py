"""Debounced start/stop push button.

The M1 build spec requires "an easily accessible start/stop button", so one
momentary button serves both roles: the first press starts the run, the next
press stops it.

A press is latched by a pin interrupt rather than by polling.  This matters:
if the main loop is ever busy for longer than the operator holds the button,
a polled read would miss the press entirely and the AGV would keep driving.
The interrupt catches the edge whatever the loop is doing.

Contact bounce is rejected by ignoring edges that arrive within
BUTTON_DEBOUNCE_MS of the last accepted one.  Because the latch is set-only,
a bounce that does slip through is harmless - it just re-sets a flag that is
already set.
"""

from machine import Pin
from utime import ticks_ms, ticks_diff, sleep_ms


class Button:
    def __init__(self, pin_no, active_high, debounce_ms):
        self._active_high = active_high
        self._debounce_ms = debounce_ms

        # Pull the line to the inactive level so a disconnected or broken
        # signal wire reads as "not pressed" rather than floating.
        pull = Pin.PULL_DOWN if active_high else Pin.PULL_UP
        self._pin = Pin(pin_no, Pin.IN, pull)

        self._last_edge_ms = ticks_ms()
        self._latched = False

        trigger = Pin.IRQ_RISING if active_high else Pin.IRQ_FALLING
        self._pin.irq(trigger=trigger, handler=self._on_edge)

    def _on_edge(self, _pin):
        """Interrupt handler - keep this short, it pre-empts the main loop."""
        now = ticks_ms()
        if ticks_diff(now, self._last_edge_ms) < self._debounce_ms:
            return                      # contact bounce, ignore
        self._last_edge_ms = now
        self._latched = True

    def is_down(self):
        """True while the button is physically held down."""
        return self._pin.value() == (1 if self._active_high else 0)

    def was_pressed(self):
        """True if a press has been latched since the last call.

        Clears the latch, so each physical press is reported exactly once.
        """
        if not self._latched:
            return False
        self._latched = False
        return True

    def clear(self):
        """Discard any latched press."""
        self._latched = False

    def wait_for_press(self):
        """Block until the button is pressed and released again.

        Only used before the run starts, where blocking is safe and in fact
        wanted - the AGV must sit still until the operator starts it.
        """
        self.clear()
        while not self.was_pressed():
            sleep_ms(10)
        # Wait out the release so the same press cannot immediately be read
        # again as the stop command.
        while self.is_down():
            sleep_ms(10)
        sleep_ms(self._debounce_ms)
        self.clear()
