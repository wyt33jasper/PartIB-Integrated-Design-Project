from machine import Pin, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep

def test_tcs3472():
    # Both options works
    # i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9))  # I2C0 on GP8 & GP9
    i2c_bus = I2C(id=0, sda=Pin(8), scl=Pin(9)) # I2C0 on GP8 & GP9
    # print(i2c_bus.scan()[0])  # Get the address (nb 41=0x29)
    tcs = tcs3472(i2c_bus)

    while True:
        print("Light:", tcs.light())
        print("RGB:", tcs.rgb())
        sleep(1)


if __name__ == "__main__":
    test_tcs3472()
