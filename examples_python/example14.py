import pyCandle
import sys
import time

# Create CANdle object and ping FDCAN bus in search of drives.
# Any found drives will be printed out by the ping() method.

candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)
ids = candle.ping()
candle.addMd80(ids[0])

candle.writeMd80Register(ids[0], pyCandle.Md80Reg_E.positionWindow, 0.05)
candle.writeMd80Register(ids[0], pyCandle.Md80Reg_E.velocityWindow, 1.0)
candle.writeMd80Register(ids[0], pyCandle.Md80Reg_E.profileAcceleration, 10.0)
candle.writeMd80Register(ids[0], pyCandle.Md80Reg_E.profileDeceleration, 5.0)
candle.writeMd80Register(ids[0], pyCandle.Md80Reg_E.profileVelocity, 15.0)
candle.writeMd80Register(ids[0], pyCandle.Md80Reg_E.quickStopDeceleration, 200.0)

candle.controlMd80SetEncoderZero(ids[0])                    #  Reset encoder at current position
candle.controlMd80Mode(ids[0], pyCandle.POSITION_PROFILE)   # Set mode to position profile
candle.controlMd80Enable(ids[0], True)                      # Enable the drive

candle.begin()

candle.md80s[0].setTargetPosition(10.0)

while not candle.md80s[0].isTargetPositionReached():
    time.sleep(1)

candle.md80s[0].setProfileAcceleration(20.0)
candle.md80s[0].setProfileVelocity(15.0)
candle.md80s[0].setTargetPosition(-10.0)

while not candle.md80s[0].isTargetPositionReached():
    time.sleep(1)

candle.end()
sys.exit("EXIT SUCCESS")
