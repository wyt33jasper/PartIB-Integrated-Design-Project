from machine import Pin, PWM
import time

class Servo:
    # Common settings for 270° servos:
    # 0.5 ms → minimum (0°)
    # 2.5 ms → maximum (270°)
    # Total span = 2.0 ms
    # Frequency: 50 Hz (20 ms period)
    # 
    # Duty % = Pulse width / Period x 100
    # Minimum (0°) = 0.5/20
    # Centre (135°) = 1.5/20
    # Maximum (270°) = 2.5/20
    #
    # Convert to Pico PWM (16-bit)
    # Pico uses: 0 → 65535
    # Duty value = Duty Fraction x 65535
    # Minimum (0.5 ms) = 0.025 * 65535
    # Centre (1.5 ms) = 0.075 * 65535
    # Maximum (2.5 ms) = 0.0125 * 65535
    MIN_DUTY = 1638 # 0°
    MID_DUTY = 4915 # 135°
    MAX_DUTY = 8192 # 270°
    
    def __init__(self, ServoLifter_pin, ServoGrabber_pin):
        self.ServoLifter_pin = Pin(ServoLifter_pin)
        self.ServoGrabber_pin = Pin(ServoGrabber_pin)
        self.ServoLifter = PWM(ServoLifter_pin)
        self.ServoGrabber = PWM(ServoGrabber_pin)
        self.ServoLifter.freq(50)  # 50 Hz for servo
        self.ServoGrabber.freq(50)
 
    # Convert angle (0–270) to duty
    def set_Lifter_angle(self, Lifter_angle):
       # Clamp angle
       if Lifter_angle < 0:
           Lifter_angle = 0
       if Lifter_angle > 90:
           Lifter_angle = 90
 
       duty = int(Servo.MIN_DUTY + (Lifter_angle / 270) * (Servo.MAX_DUTY - Servo.MIN_DUTY))
       self.ServoLifter.duty_u16(duty)
       
    def set_Grabber_angle(self, Grabber_angle):
       # Clamp angle
       if Grabber_angle < -10:
           Grabber_angle = -10
       if Grabber_angle > 80:
           Grabber_angle = 80
 
       duty = int(Servo.MIN_DUTY + (Grabber_angle / 270) * (Servo.MAX_DUTY - Servo.MIN_DUTY))
       self.ServoGrabber.duty_u16(duty)