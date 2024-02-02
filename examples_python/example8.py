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

candle.controlMd80SetEncoderZero(ids[0])       #  Reset encoder at current position

candle.controlMd80Mode(ids[0], pyCandle.POSITION_PID)     # Set mode to position PID control
candle.controlMd80Enable(ids[0], True)    # Enable the drive

#  We will run both Position PID and Velocity PID at default settings. If you wish you can play with the parameters
#  Using the methods below:
# candle.md80s[0].setPositionControllerParams(20.0, 0.2, 0.0, 15.0)
# candle.md80s[0].setVelocityControllerParams(0.0, 0.1, 0.0, 1.5)
# candle.md80s[0].setMaxTorque(0.5)

#  To reload default controller parameters, simply disable the drive (contorlMd80Enable(id, false)), 
#  stop the communications (candle.end()) or power cycle the drive (off-on).

t = 0.0
dt = 0.02

# Begin update loop (it starts in the background)
candle.begin()

for i in range(1000):
    t = t  + dt
    candle.md80s[0].setTargetPosition(math.sin(t) * 2.0)  
    print("Drive ID = " + str(candle.md80s[0].getId()) + " Velocity: "  + str(candle.md80s[0].getVelocity()))
    time.sleep(0.01)  # Add some delay

# Close the update loop
candle.end()

sys.exit("EXIT SUCCESS") 
