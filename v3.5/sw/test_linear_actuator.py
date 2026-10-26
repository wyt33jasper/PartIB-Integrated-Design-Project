from machine import Pin, PWM
from utime import sleep

class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
           
    def set(self, dir, speed):
        self.mDir.value(dir)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor


def test_actuator1():
    actuator1 = Actuator(dirPin=0, PWMPin=1)  # Actuator 1 controlled from Motor Driv1 #1, which is on GP0/1

    while True:
        print("Extending quickly")
        actuator1.set(dir = 0, speed=100)
        sleep(5)  # nb we don't know when this has finished without another means

        print("Retracing slowly")
        actuator1.set(dir=1, speed=25)
        sleep(10)  # nb we don't know when this has finished without another means


if __name__ == "__main__":
    test_actuator1()
