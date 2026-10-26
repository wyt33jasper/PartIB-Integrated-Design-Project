from machine import Pin, SoftI2C, I2C
from libs.mfrc522_python.src.mfrc522.MFRC522 import MFRC522
from utime import sleep, ticks_ms

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

        # Select the RFID tag
        mfrc522.SelectTag(uid)

        # Now read every sector
        start_time = ticks_ms()
        data = []
        for sector in range(16):
            # Authenticate
            status = mfrc522.Authenticate(MFRC522.PICC_AUTHENT1A, (sector * 4), KEY, uid)
            if status != MFRC522.MI_OK:
                print(f"Error with Authentication of sector={sector}, status={status}")
                continue

            try:
                data.append([])
                data[sector] = []
                # Read data blocks specified by block_addr
                for block in range(4):
                    data[sector].append(mfrc522.ReadTag(sector * 4 + block))

            except Exception as e:
                 print(f"Read exception: {e}")
                 break
        end_time = ticks_ms()

        for sector in range(16):
            for block in range(4):
                if sector == 0 and block == 0:  # Manufacturer Block
                    # Bytes 0-3 are UID, with bytes 4 as a checksum
                    uid = data[sector][block][0:4]
                    checksum = data[sector][block][4]

                    # Check if UID is valid
                    uidCheck = 0
                    for i in range(4):
                        uidCheck = uidCheck ^ uid[i]
                    if uidCheck != checksum:
                        print(f"UID in card didn't match checksum uid={' '.join(hex(i) for i in uid)}, checksum={checksum}}")
                    else:
                        # Bytes 5-15 are Manufacturer Data
                        manufacturerData = data[sector][block][5:16]

                        print(f"  Sector={sector}, block={block}, Manufacturer - uid={' '.join(hex(i) for i in uid)}, manufacturerData={' '.join(hex(i) for i in manufacturerData)}")
                elif block == 3: # Sector Trailer
                    # Decode Sector Trailer
                    keyA = data[sector][block][0:6]
                    accessBits = data[sector][block][6:10] # For more info, see section 8.7 here https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf 
                    keyB = data[sector][block][10:16]
                    print(f"  Sector={sector}, block={block}, Trailer - keyA={' '.join(hex(i) for i in keyA)}, keyB={' '.join(hex(i) for i in keyB)}, accessBits={' '.join(hex(i) for i in accessBits)}")
                else:
                    print(f"  Sector={sector}, block={block}, Data - data={' '.join(hex(i) for i in data[sector][block])}")
        print(f" Reading took {end_time - start_time} ms")

        # Read just one block
        sector = 3
        block = 0
        data = None
        start_time = ticks_ms()
        # Authenticate
        status = mfrc522.Authenticate(MFRC522.PICC_AUTHENT1A, (sector * 4), KEY, uid)
        if status != MFRC522.MI_OK:
            print(f"Error with Authentication of sector={sector}, status={status}")
        else:
            try:
                data = mfrc522.ReadTag(sector * 4 + block)
            except Exception as e:
                 print(f"Read exception: {e}")
        end_time = ticks_ms()
        if data is not None:
            print(f"Read in {end_time - start_time} ms Sector={sector}, block={block}, data={' '.join(hex(i) for i in data)}")
            # print(f"data_ascii={''.join(chr(i) for i in data)}")

        print("Done...")

        # Stop cryptographic communication with the tag
        mfrc522.StopCrypto1()

        sleep(4)  # Allow time to remove the card!


if __name__ == "__main__":
    test_mfrc522()
