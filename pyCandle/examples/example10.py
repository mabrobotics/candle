import mab.pyCandle as pyCandle
import time
import math
import sys  


motor_id = 104
# Create all candles object and add one motor
handler = pyCandle.MultipleCandles(False) 
handler.addMd80([motor_id])
handler.setModeMd80([motor_id], "IMPEDANCE")
handler.setImpedanceParamters({motor_id: {"kp": 50, "kd": 1.0, "max_torque": 30}})
handler.enableAllMotors()


command = {motor_id: {"position": 0.0, "velocity": 0.0, "torque": 0.0}}
t = 0.0
dt = 0.04

for i in range(1000):
    # command[100]["position"] = math.sin(t)*2.0
    handler.sendMotorCommand(int(t), command)
    print(handler.getAllMotorsData())
    t = t + dt
    time.sleep(0.01)  # Add some delay


# Close the update loop
handler.disableAllMotors()

sys.exit("EXIT SUCCESS")
