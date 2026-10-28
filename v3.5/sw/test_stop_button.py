"""Bench test for the start/stop button and the amber beacon.

Run this before plugging the motors in.  It exercises exactly the logic
main.py uses, but drives nothing, so a mis-wired button cannot send the AGV
across the table.

Expected behaviour:
  1. "Press the button to start" is printed and nothing flashes.
  2. On the first press the beacon starts flashing at 2 Hz and the loop
     counter is printed once a second.
  3. On the next press the beacon goes dark and the test reports how long the
     loop took to notice - this should be a handful of milliseconds.

If the beacon starts flashing on its own, BUTTON_ACTIVE_HIGH in config.py is
set the wrong way round.  If one press registers as two, raise
BUTTON_DEBOUNCE_MS.
"""

from utime import sleep_ms, ticks_ms, ticks_diff

import config
from drivers.button import Button
from drivers.indicators import FlashingLamp


def test_stop_button():
    button = Button(config.BUTTON_PIN,
                    active_high=config.BUTTON_ACTIVE_HIGH,
                    debounce_ms=config.BUTTON_DEBOUNCE_MS)
    beacon = FlashingLamp(config.AMBER_LED_PIN, config.AMBER_FLASH_MS)

    print("Press the button to start")
    button.wait_for_press()
    print("Running - press again to stop")

    beacon.on()
    passes = 0
    started_ms = ticks_ms()
    last_report_ms = started_ms

    try:
        while True:
            if button.was_pressed():
                break

            beacon.update()
            passes += 1

            now = ticks_ms()
            if ticks_diff(now, last_report_ms) >= 1000:
                print(f"running, {passes} loop passes")
                last_report_ms = now

            sleep_ms(config.LOOP_PERIOD_MS)
    finally:
        beacon.off()

    elapsed_ms = ticks_diff(ticks_ms(), started_ms)
    print(f"Stopped after {elapsed_ms} ms and {passes} loop passes")
    print(f"Average loop period = {elapsed_ms / max(passes, 1):.1f} ms")


if __name__ == "__main__":
    test_stop_button()
