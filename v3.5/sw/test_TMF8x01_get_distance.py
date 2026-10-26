'''
  @file demo_get_distance.py
  @brief Get measurement data by PROXIMITY and DISTANCE hybrid mode.
  @note  TMF8801 only suport one mode, PROXIMITY and DISTANCE hybrid mode.
  @n 
  @n --------------------------------------------------------------------------------|
  @n |  Type     |   suport ranging mode     |  ranging ranges |  Accuracy           |
  @n |---------------------------------------|-----------------|---------------------|
  @n |  TMF8801  | PROXIMITY and DISTANCE    |                 |  20~100mm: +/-15mm  |
  @n |           |  hybrid mode(only one)    |    20~240cm     |  100~200mm: +/-10mm |
  @n |           |                           |                 |   >=200: +/-%5      |
  @n |---------------------------------------|-----------------|---------------------|
  @n |           |     PROXIMITY mode        |    0~10cm       |                     |
  @n |           |---------------------------|-----------------|   >=200: +/-%5      |
  @n |  TMF8701  |     DISTANCE mode         |    10~60cm      |  100~200mm: +/-10mm |
  @n |           |---------------------------|-----------------|                     | 
  @n |           | PROXIMITY and DISTANCE    |    0~60cm       |                     |
  @n |           |      hybrid mode          |                 |                     |
  @n |---------------------------------------|-----------------|----------------------
  @n 
  @n hardware conneted table:
  @n ------------------------------------------
  @n |  TMF8x01  |            MCU              |
  @n |-----------------------------------------|
  @n |    I2C    |       I2C Interface         |
  @n |-----------------------------------------|
  @n |    EN     |   not connected, floating   |
  @n |-----------------------------------------|
  @n |    INT    |   not connected, floating   |
  @n |-----------------------------------------|
  @n |    PIN0   |   not connected, floating   |
  @n |-----------------------------------------|
  @n |    PIN1   |    not connected, floating  |
  @n |-----------------------------------------|
  @n
  @Copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author [Arya](xue.peng@dfrobot.com)
  @version  V1.0
  @date  2021-04-06
  @url https://github.com/DFRobot/DFRobot_TMF8x01
'''

from utime import sleep
from machine import Pin, SoftI2C, I2C

from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8801, DFRobot_TMF8701

def test_TMF8x01_get_distance():
    # Both options work
    # i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9), freq=100000)  # I2C0 on GP8 & GP9
    i2c_bus = I2C(id=0, sda=Pin(8), scl=Pin(9), freq=100000) # I2C0 on GP8 & GP9
    #print(i2c_bus.scan()) # 65=0x41
    assert len(i2c_bus.scan()) == 1 # This demo requires exactly one device


    # Set the correct device
    device = "TMF8701"


    # Use the correct one - TODO can we auto detect this?
    if device == "TMF8701":
      tof = DFRobot_TMF8701(i2c_bus=i2c_bus)
    elif device == "TMF8801":
      tof = DFRobot_TMF8801(i2c_bus=i2c_bus)
    else:
       raise RuntimeError(f"Device {device} not known")

    print("Initialising ranging sensor TMF8x01......")
    while(tof.begin() != 0):
      print("   Initialisation failed")
      sleep(0.5)
    print("   Initialisation done.")

    print("Software Version: ", end=" ")
    print(tof.get_software_version())
    print("Unique ID: %X"%tof.get_unique_id())
    print("Model: ", end=" ")
    print(tof.get_sensor_model())

    '''
    @brief Config measurement params to enable measurement. Need to call stop_measurement to stop ranging action.
    @param calib_m: Is an enumerated variable of , which is to config measurement cailibration mode.
    @n     eMODE_NO_CALIB  :          Measuring without any calibration data.
    @n     eMODE_CALIB    :          Measuring with calibration data.
    @n     eMODE_CALIB_AND_ALGOSTATE : Measuring with calibration and algorithm state.
    @param mode : the ranging mode of TMF8701 sensor.
    @n     ePROXIMITY: Raing in PROXIMITY mode,ranging range 0~10cm
    @n     eDISTANCE: Raing in distance mode,ranging range 10~60cm
    @n     eCOMBINE:  Raing in PROXIMITY and DISTANCE hybrid mode,ranging range 0~60cm
    @return status:
    @n      false:  enable measurement failed.
    @n      true:  enable measurement sucess.
    '''

    if device == "TMF8701":
      tof.start_measurement(calib_m = tof.eMODE_NO_CALIB, mode = tof.ePROXIMITY)
      #tof.start_measurement(calib_m = tof.eMODE_NO_CALIB, mode = tof.eCOMBINE)
      #tof.start_measurement(calib_m = tof.eMODE_NO_CALIB, mode = tof.eDISTANCE)
    elif device == "TMF8801":
      tof.start_measurement(calib_m = tof.eMODE_NO_CALIB)
    else:
       raise RuntimeError(f"Device {device} not known")

    while True:
      if(tof.is_data_ready() == True):
        print(f"Distance = {tof.get_distance_mm()} mm{" (For TMF8701, make sure you read about mode selection above!)" if device == "TMF8701" else ""}")
      sleep(0.5)


if __name__ == "__main__":
    test_TMF8x01_get_distance()