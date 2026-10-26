from time import sleep
from machine import Pin, I2C

from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader

def test_tiny_code_reader():
    print("Starting tiny code reader...")

    # Set up for the Pico, pin numbers will vary across boards.
    i2c_bus = I2C(id=0, scl=Pin(17), sda=Pin(16), freq=400000) # I2C0 on GP16 & GP17

    i2c_devs = i2c_bus.scan()
    # Uncomment this to see what peripherals were detected on the bus. We would
    # expect to see [12] as the output, since that's the sensor's ID.
    #print(i2c_devs)
    assert len(i2c_devs) == 1 # This demo requires exactly one device
    assert i2c_devs[0] == 12 # Expected device

    tiny_code_reader = TinyCodeReader(i2c_bus)

    print("Polling!")

    # Keep looping and reading the sensor - a real application may do this in
    # a separate thread or a few times when it expects to find a QR code
    while True:
        sleep(TinyCodeReader.TINY_CODE_READER_DELAY)

        code = tiny_code_reader.poll()
        if code is not None:
            print(f"Code found: {code}")


if __name__ == "__main__":
    test_tiny_code_reader()