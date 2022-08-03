import mab.pyCandle as pyCandle
import time
import math
import sys  

motor_id = 101
candle = None
while True:
    try:
        tmp = pyCandle.Candle(pyCandle.CAN_BAUD_1M,True)
        ids = tmp.ping()
        print(ids)
        if motor_id in ids:
            candle = tmp
            break
    except:
        break


candle.addMd80(motor_id, False)

for md in candle.md80s:
    candle.controlMd80Mode(md, pyCandle.IMPEDANCE)    # Set mode to impedance control
    md.setImpedanceControllerParams(100.0, 2.5)
    md.setMaxTorque(30)
    candle.controlMd80Enable(md, True)     # Enable the drive

t = 0.0
dt = 0.04

# Begin update loop (it starts in the background)
candle.begin()
count = 0
# for i in range(1000):
while True:
    # Once again we loop over all drives, this time setting thier position target. All drives should now perform
    # a nice synchronized movement.
    for md in candle.md80s:
        # if count % 2 == 0:
        #     md.setTargetPosition(0.0)
        # else:
        #     md.setTargetPosition(0.1)
        print(md.getMotorStatus())

    count += 1;
        
    t = t + dt
    time.sleep(5)  # Add some delay


# Close the update loop
candle.end()

sys.exit("EXIT SUCCESS")
