"""Bench test for the QR code reader.

Run this with the AGV on the bench and a printed QR code in hand. It prints
every confirmed target and reports the working distance, which the PicoBot
guide asks the team to find - the author's sample read reliably at about
180 mm, and every reader is slightly different.

Expected behaviour:
  1. "scanning" prints and nothing else happens with no code in view.
  2. Bring a code into view and a target prints once, not repeatedly.
  3. Take it away and bring it back, and it prints once more.

If "no I2C device found" appears, the reader is unplugged or on the wrong
pins. If devices are listed but 12 is not among them, the address is wrong.

Move the code slowly towards and away from the reader and note where it
stops decoding; those two distances are the mounting limits.
"""

from machine import I2C, Pin
from utime import sleep_ms, ticks_ms, ticks_diff

import config
from drivers.qr import QRReader
from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader


def test_qr_reader():
    i2c = I2C(id=config.QR_I2C_ID,
              sda=Pin(config.QR_SDA_PIN),
              scl=Pin(config.QR_SCL_PIN),
              freq=config.QR_I2C_FREQ)

    devices = i2c.scan()
    if not devices:
        print("no I2C device found - check wiring and pins")
        return
    print("I2C devices:", devices)
    if TinyCodeReader.TINY_CODE_READER_I2C_ADDRESS not in devices:
        print("code reader (12) not on the bus - check the address")
        return

    reader = QRReader(TinyCodeReader(i2c),
                      poll_ms=config.QR_POLL_MS,
                      confirm_reads=config.QR_CONFIRM_READS)

    print("scanning - hold a QR code in front of the reader")

    last_state = None
    found = 0
    while True:
        target = reader.poll()

        if target is not None:
            found += 1
            print("target {}: {}".format(found, target))
            # Re-arm so the next package is read as a new one.
            reader.reset()

        # Show progress towards a confirmation, so it is obvious when the
        # reader is seeing a code but not holding it steadily enough.
        state = (str(reader.reading), reader.confirmations, reader.error_count)
        if state != last_state:
            last_state = state
            if reader.reading is not None:
                print("  seeing {} ({}/{})".format(
                    reader.reading, reader.confirmations,
                    config.QR_CONFIRM_READS))
            elif reader.error_count:
                print("  I2C errors: {}".format(reader.error_count))

        sleep_ms(config.LOOP_PERIOD_MS)


if __name__ == "__main__":
    test_qr_reader()
