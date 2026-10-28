from machine import Pin, SoftI2C, I2C
from libs.mfrc522_python.src.mfrc522.MFRC522 import MFRC522
from utime import sleep, ticks_ms

def check_block_valid_for_write(sector, block, write_trailer=False):
    if sector == 0 and block == 0:
        raise RuntimeError("Cannot Write to Manufacturer Block")
    elif not write_trailer and block == 3:
        raise RuntimeError("Writing to the trailer block could force encryption, be sure you know what you're doing")

def test_mfrc522():
    # Both options works
    # i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9))  # I2C0 on GP8 & GP9
    i2c_bus = I2C(id=0, sda=Pin(8), scl=Pin(9)) # I2C0 on GP8 & GP9
    print(i2c_bus.scan())  # Get the address (nb 40=0x28)

    mfrc522 = MFRC522(i2c_bus)
    KEY=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]

    while True:
        # Send request to RFID tag
        (status, TagType) = mfrc522.Request(MFRC522.PICC_REQIDL)
        if status != MFRC522.MI_OK:
            # Error expected if no tag
            if status == MFRC522.MI_ERR:
                print("Not detected")
                sleep(0.5)
            else:
                print(f"Error requesting PIC_REQIDL status={status}")
            continue

        # Anticollision, return UID if successful
        (status, uid) = mfrc522.Anticoll()
        if status != MFRC522.MI_OK:
            print(f"Error with anticollison status={status}")
            continue

        print(f"Found ID: uid={' '.join(hex(i) for i in uid)}")
        print("Write to this card? <y/N>")
        check = input()
        if check != "y":
            print("Aborting")
            continue

        # Select the RFID tag
        mfrc522.SelectTag(uid)

        # Write data to just one block
        sector = 3
        block = 0
        data = "Hidden message!!"
        if len(data) != 16:
            raise RuntimeError("Need len 16 not {len(data)} to program a block")
        check_block_valid_for_write(sector=sector, block=block)

        # Authenticate
        start_time = ticks_ms()
        status = mfrc522.Authenticate(MFRC522.PICC_AUTHENT1A, (sector * 4), KEY, uid)
        if status != MFRC522.MI_OK:
            print(f"Error with Authentication of sector={sector}, status={status}")
        else:
            # Write the block
            try:
                mfrc522.WriteTag(sector * 4 + block, [ord(c) for c in data])
            except Exception as e:
                 print(f"Write exception: {e}")
                 mfrc522.StopCrypto1()
                 continue

            # Read the block to check:
            dataRead = None
            try:
                dataRead = mfrc522.ReadTag(sector * 4 + block)
            except Exception as e:
                 print(f"Read exception: {e}")
        end_time = ticks_ms()
        if [ord(c) for c in data] == dataRead:
            print(f"Write and check done in {end_time - start_time} ms")
        else:
            print(f"Data Mismatch in={data}, out={dataRead}")

        # Stop crypto
        mfrc522.StopCrypto1()

        print("Done...")


if __name__ == "__main__":
    test_mfrc522()
