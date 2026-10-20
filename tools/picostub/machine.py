"""Host-side stand-in for MicroPython's machine module.

Only exists so that the real sw/ code can be imported and exercised on a PC.
Never uploaded to the Pico - the Pico has the real thing.
"""


class Pin:
    IN = 0
    OUT = 1
    PULL_UP = 2
    PULL_DOWN = 3
    IRQ_RISING = 4
    IRQ_FALLING = 5

    registry = {}

    def __init__(self, no, mode=None, pull=None):
        self.no = no
        self._v = 0
        self._handler = None
        Pin.registry[no] = self

    def value(self, v=None):
        if v is None:
            return self._v
        self._v = v

    def irq(self, trigger=None, handler=None):
        self._handler = handler

    def press(self):
        """Simulate a button press edge."""
        self._v = 1
        if self._handler:
            self._handler(self)
        self._v = 0


class PWM:
    def __init__(self, pin, freq=None):
        self.pin = pin
        self.duty = 0

    def freq(self, f):
        pass

    def duty_u16(self, d):
        self.duty = d


class ADC:
    """Host-side stand-in for machine.ADC.

    Tests set `value_u16` to pretend the Hall sensor is seeing a field.
    """

    registry = {}

    def __init__(self, pin):
        self.pin = pin
        self.value_u16 = 32768          # mid scale, as a ratiometric part idles
        ADC.registry[getattr(pin, "no", pin)] = self

    def read_u16(self):
        return self.value_u16
