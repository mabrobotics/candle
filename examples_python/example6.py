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

# Now we shall loop over all found drives to change control mode and enable them one by one
for md in candle.md80s:
    candle.controlMd80SetEncoderZero(md)      #  Reset encoder at current position
    candle.controlMd80Mode(md, pyCandle.IMPEDANCE)    # Set mode to impedance control
    candle.controlMd80Enable(md, True)     # Enable the drive

t = 0.0
dt = 0.04

# Begin update loop (it starts in the background)
candle.begin()

for i in range(1000):
    # Once again we loop over all drives, this time setting thier position target. All drives should now perform
    # a nice synchronized movement.
    for md in candle.md80s:
        md.setTargetPosition(math.sin(t) * 2.0)

    t = t + dt
    time.sleep(0.01)  # Add some delay


# Close the update loop
candle.end()

sys.exit("EXIT SUCCESS")
