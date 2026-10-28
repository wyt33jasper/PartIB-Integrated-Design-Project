from machine import Pin, ADC
from utime import sleep

def test_input_poll():
    "Simple poll of input"
    input_pin = 28
    input = ADC(Pin(input_pin)) # Think carefully whether you need pull up or pull down

    while True:
        # Poll the value
        value = input.read_u16()
        print(f"Input = {value}")
        sleep(0.2)


def input_irq(p):
    "Interrupt handler"
    # print(p)
    value = p.value()
    print(f"Input changed, value={value}")


def test_input_irq():
    "More advanced, interrupt based input handling"
    input_pin = 18  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down
    input.irq(handler=input_irq) # Register irq, you could also consider rising and falling edges c.f. https://docs.micropython.org/en/latest/library/machine.Pin.html

    while True:
        pass # irq handling does the rest in this instance


if __name__ == "__main__":
    test_input_poll()
    # test_input_irq()