from machine import Pin, PWM
from utime import sleep

def test_pwm():
    pwm_pin_no = 28  # Pin 28 = GP28 (labelled 34 on the jumper)
    pwm_pin = PWM(Pin(pwm_pin_no), 100)

    level = 0  # 0-100
    direction = 1  # 1=up, -1=down

    while True:
        # PWM the specified pin
        u16_level = int(65535 * level / 100)
        pwm_pin.duty_u16(u16_level)
    
        # update level and sleep
        print(f"Level={level}, u16_level={u16_level}, direction={direction}")
        level += direction
        if level == 100:
            direction = -1
        elif level == 0:
            direction = 1
        sleep(0.1)


if __name__ == "__main__":
    test_pwm()
