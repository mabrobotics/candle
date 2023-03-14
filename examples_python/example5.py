import pyCandle
import time
import math
import sys  


def _range(x, min, max):

    if x < min:
        return min
    if x > max:
        return max
    return x


kp = 0.0
kd = 0.0

if len(sys.argv) == 3:
    kp = float(sys.argv[1])
    kd = float(sys.argv[2])
    kp = _range(kp, 0, 5.0)
    kd = _range(kd, 0, 0.5)
    print("Arguments incorrect!")
else:
    print("Not correct arguments!")
    print("Arguments are: kp kd")
    print("kp = <0, 5.0>, kd = <0, 0.5f>")
    print("For example:")
    print("python3 ./example5.py 1.0 0.1")
    sys.exit("EXIT FALIURE") 
    

# Create CANdle object and set FDCAN baudrate to 1Mbps
candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)

# Ping FDCAN bus in search of drives
ids = candle.ping()

if len(ids) == 0: # If no drives found -> quit
    sys.exit("EXIT FALIURE") 

# Add all found to the update list
for id in ids:
    candle.addMd80(id)

candle.controlMd80SetEncoderZero(ids[0]);                #  Reset encoder at current position

candle.controlMd80Mode(ids[0], pyCandle.IMPEDANCE)       # Set mode to impedance control
candle.controlMd80Enable(ids[0], True)                   # Enable the drive

#  Now we modify the Impedance controller parameters - the drive will behave much different than in 
#  previous examples. The drive will change default params to the ones we select below.
candle.md80s[0].setImpedanceControllerParams(kp, kd)

#  To reload default controller parameters, simply disable the drive (contorlMd80Enable(id, false)), 
#  stop the communications (candle.end()) or power cycle the drive (off-on).

t = 0.0
dt = 0.04

# Begin update loop (it starts in the background)
candle.begin()

for i in range(1000):

    t = t + dt
    candle.md80s[0].setTargetPosition(math.sin(t))
    time.sleep(0.01)  # Add some delay


# Close the update loop
candle.end()

sys.exit("EXIT SUCCESS") 
