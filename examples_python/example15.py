import pyCandle
import time
import math
import sys


def getExampleConfirmation():
    print("This example will spin the motor very fast if the output shaft is unloaded.")
    x = input("Are you sure you want to proceed? [Y/n]").upper()
    if x != "Y":
        print("Confirmation failed.")
        return False
    return True


# Create CANdle object and set FDCAN baudrate to 1Mbps
candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)

# Ping FDCAN bus in search of drives
ids = candle.ping()

if len(ids) == 0 or not getExampleConfirmation():  # If no drives found -> quit
    sys.exit("EXIT FALIURE")

# Add all found to the update list
for id in ids:
    candle.addMd80(id)

candle.controlMd80SetEncoderZero(ids[0])
candle.controlMd80Mode(ids[0], pyCandle.RAW_TORQUE)
candle.controlMd80Enable(ids[0], True)

candle.begin()
candle.md80s[0].setTargetTorque(0.1)
time.sleep(5)
candle.end()

sys.exit("EXIT SUCCESS")
