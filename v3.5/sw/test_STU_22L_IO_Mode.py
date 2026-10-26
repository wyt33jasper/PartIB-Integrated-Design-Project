from machine import Pin
from utime import sleep

def test_STU_22L_IO_Mode():
    # STU-22L data sheet referenced here: https://robu.in/wp-content/uploads/2023/08/1666947-LDROBOT_STU-22L_Datasheet.pdf
    # Here using 5.4(1) anologue on the TX pin - IO Mode:
    #    The response time for judgment in IO mode is 10ms；
    #    When the target material is a soft material, the TX pin outputs a low level of 0V;
    #    When the target material is a hard material, the TX pin outputs a high level of 3.3V.
    STU_22L_TX_pin = 28  # Pin 28 = GP28 (labelled 34 on the jumper)
    stu_22l = Pin(STU_22L_TX_pin, Pin.IN)

    while True:
        # Read the value
        val = stu_22l.value()
        print(f"""STU-22L val={val} => {"hard" if val == 1 else "soft"}""")
        sleep(0.5)

if __name__ == "__main__":
    test_STU_22L_IO_Mode()