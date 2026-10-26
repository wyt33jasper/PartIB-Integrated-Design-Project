from machine import Pin, UART
from utime import sleep

def test_STU_22L_UART():
    # STU-22L info referenced here: https://wiki.dfrobot.com/SKU_SEN0560_Ultrasonic_Material_Detection_Sensor
    # Here using 5.4(2) - UART Mode:
    #    Module is powered on, it sends instructions to the module through the serial port to switch from IO
    #    mode to serial port mode. After that, the module sends related data at a frequency of 100 Hz.
    #    This data follows the module's serial port software communication protocol.
    #
    #    Baud Rate  Data Bits  Stop Bits  Parity Bit  Flow Control
    #      115200	    8	       1          No           No

    # nb Connect device's TX to Pico's RX (and vice versa for RX/TX)
    # UART0 on pins GP17/GP16 (pins 21/22 on the jumper)
    UART_id = 0
    UART_tx = Pin(16)
    UART_rx = Pin(17)
    baud = 115200
    bits = 8
    stop = 1
    parity = None
    timeout_ms = 10

    print("Writing to STU-22L Device...")
    # nb We limit rxbuf to 1, to try to get rid of data in between reads
    uart = UART(UART_id, baudrate=baud, bits=bits, parity=parity, stop=stop, tx=UART_tx, rx=UART_rx, timeout=timeout_ms, rxbuf=1)

    # Configure device in serial mode
    data = [0XAA, 0XAA, 0XFE, 0X01, 0X00, 0X53]
    uart.write(bytes(data))
    print("Written to STU-22L Device")

    while True: # Outer loop - reading data
        try:
            while True: # Inner loop - getting a valid sample
                checksum = int.from_bytes(uart.read(1))
                if checksum == 0xAA:
                    data = uart.read(7)

                    for d in data[0:6]:
                        checksum += d
                    checksum &= 0xFF
                    if checksum == data[6]:
                        # When hard material is detected, data[3] == 0; when soft material is detected, data[3] == 1.
                        print(f"STU-22L val={data[3]} => {"hard" if int(data[3]) == 0 else "soft"} (in my testing this gave huge latency, IO mode might be better)""")
                    else:
                        # Must be mis-synchronised with UART stream or glitch on bus
                        pass
                    break # Break to outer loop
                else:
                    # Must be mis-syncrhonised with UART stream
                    pass
        except Exception as e:
            print(f"STU_22L UART gave exception: {e}")
            raise

        sleep(0.5)

if __name__ == "__main__":
    test_STU_22L_UART()