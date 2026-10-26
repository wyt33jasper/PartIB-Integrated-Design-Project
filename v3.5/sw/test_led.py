from machine import Pin
from utime import sleep

def test_led():
    led_pin = 28  # Pin 28 = GP28 (labelled 34 on the jumper)
    led = Pin(led_pin, Pin.OUT)

    while True:
        # Flash the LED
        print("Flashing LED")
        led.value(1)
        sleep(0.5)
        led.value(0)
        sleep(0.5)

if __name__ == "__main__":
    test_led()
