from re import T
import mab.pyCandle as pyCandle
import time
import math
import sys  


motor_id = [100,101,102,103,104,105, 106, 107,108,109, 110,111]
impedance_params = dict()

init_pos = {104: 0.48, 105: 0.0, 102: -0.0, 101: -0.148, 100: 0.0, 
            103: 0.0, 110: -0.48, 111: -0.0, 108: 0.0, 107: 0.148, 106: 0.0, 109: -0.0}

for i in motor_id:
    impedance_params[i] = {"kp": 70, "kd": 1.0, "max_torque": 30} 
# Create all candles object and add one motor
handler = pyCandle.MultipleCandles(False) 
handler.addMd80(motor_id)
handler.setModeMd80(motor_id, "IMPEDANCE")
handler.setImpedanceParamters(impedance_params)
handler.enableAllMotors()
time.sleep(0.3)
motor_command = dict()
for i, pos in init_pos.items():
    motor_command[i] = dict()
    motor_command[i]["position"] = pos
    motor_command[i]["velocity"] = 0.0
    motor_command[i]["torque"] = 0.0
    motor_command[i]["kp"] = 50.0
    motor_command[i]["kd"] = 1.0
print(handler.getAllMotorsData())
handler.sendMotorCommand(-1, motor_command)


# command = {motor_id: {"position": 0.0, "velocity": 0.0, "torque": 0.0}}
t = 0.0
dt = 0.04

while True:
    # command[100]["position"] = math.sin(t)*2.0
    handler.sendMotorCommand(int(t), motor_command)
    print(handler.getAllMotorsData())
    t = t + dt
    time.sleep(0.01)  # Add some delay


# Close the update loop
handler.disableAllMotors()

sys.exit("EXIT SUCCESS")
