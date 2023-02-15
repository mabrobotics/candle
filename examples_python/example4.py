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

# Reset encoder at current position
candle.controlMd80SetEncoderZero(ids[0])       
# Set mode to impedance control
candle.controlMd80Mode(ids[0], pyCandle.IMPEDANCE)     
# Enable the drive
candle.controlMd80Enable(ids[0], True);     

t = 0.0
dt = 0.04

# Begin update loop (it starts in the background)
candle.begin()

for i in range(1000):
    t=t+dt
    #  After powerup the drive will load set of default parameters for every controller.
    #  To make the drive move all we got to do is set mode (.controlMd80Mode) and set target
    #  (.setTargetPosition, .setTargetVelocity, .setTargetTorque)
    candle.md80s[0].setTargetPosition(math.sin(t) * 3.0)
    time.sleep(0.01)  # Add some delay

# Close the update loop
candle.end()

sys.exit("EXIT SUCCESS")
