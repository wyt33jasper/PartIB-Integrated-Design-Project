from libs.VNCL4010.VNCL4010 import VCNL4010
from machine import I2C, Pin

import time  

i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=100000)

 
sensor = VCNL4010(i2c)

 

while True:

    prox = sensor.read_proximity()

    print("Proximity:", prox,)

    time.sleep(0.2)

 