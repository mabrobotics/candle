from re import T
import mab.pyCandle as pyCandle
import time
import math
import sys  


motor_id = [100,101,102,103,104,105, 106, 107,108,109, 110,111]
impedance_params = dict()
pid_controll = {104: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 50.0}, 105: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 10.0}, 102: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 50.0}, 101: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 30.0}, 100: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 30.0}, 103: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 50.0}, 110: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 50.0}, 111: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 10.0}, 108: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 50.0}, 107: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 30.0}, 106: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 30.0}, 109: {'p_iwu': 1.0, 'p_kd': 2.5, 'p_ki': 2.9, 'p_kp': 100, 'max_vel': 2.0, 'v_iwu': 1.5, 'v_kd': 0.0, 'v_ki': 0.3, 'v_kp': 20.8, 'max_torque': 50.0}}
init_pos = {104: 0.48, 105: 0.0, 102: -0.0, 101: -0.148, 100: 0.0, 
            103: 0.0, 110: -0.48, 111: -0.0, 108: 0.0, 107: 0.148, 106: 0.0, 109: -0.0}

for i in motor_id:
    impedance_params[i] = {"kp": 70, "kd": 1.0, "max_torque": 30} 
# Create all candles object and add one motor
handler = pyCandle.MultipleCandles(False) 
handler.addMd80(motor_id)
handler.setModeMd80(motor_id, "POSITION_PID")
handler.setPositionPIDParameters(pid_controll)
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
    # handler.sendMotorCommand(int(t), motor_command)
    print(handler.getAllMotorsData())
    t = t + dt
    time.sleep(0.01)  # Add some delay


# Close the update loop
handler.disableAllMotors()

sys.exit("EXIT SUCCESS")
