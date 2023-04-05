import pyCandle
import time
import math
import sys  

# Create CANdle object and set FDCAN baudrate to 1Mbps
candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)

# Ping FDCAN bus in search of drives
ids = candle.ping()

if len(ids) == 0: # If no drives found -> quit
    sys.exit("EXIT FALIURE") 

# Add all found to the update list
for id in ids:
    candle.addMd80(id)

candle.controlMd80SetEncoderZero(ids[0])               #  Reset encoder at current position

candle.controlMd80Mode(ids[0], pyCandle.VELOCITY_PID)  # Set mode to velocity control
candle.controlMd80Enable(ids[0], True)                 # Enable the drive

#  Uncomment the line below to change the *.cfg file defaults
# candle.md80s[0].setVelocityControllerParams(0.05, 0.25, 0.0, 1.0)

#  To reload default controller parameters, simply disable the drive (contorlMd80Enable(id, false)), 
#  stop the communications (candle.end()) or power cycle the drive (off-on).

t = 0.0
dt = 0.04

# Begin update loop (it starts in the background)
candle.begin()

targetVelocity = 20.0

for i in range(2000):
    if i % 200 == 0:
        targetVelocity = targetVelocity + 1.0
    t = t + dt
    candle.md80s[0].setTargetVelocity(targetVelocity)
    print("Drive ID = " + str(candle.md80s[0].getId()) + " Velocity: " +  str(candle.md80s[0].getVelocity()))
    time.sleep(0.01)  # Add some delay

# Close the update loop
candle.end()

sys.exit("EXIT SUCCESS") 
