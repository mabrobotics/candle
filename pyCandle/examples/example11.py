from re import T
import mab.pyCandle as pyCandle
import time
import math
import sys  
'''AK70-10'''
watchdog_config = {"kp": 0.0,
                   "kd": 200.0,
                   "max_position": 3.0,
                   "min_position": 3.0,
                   "pos_percent_wanted": 0.95,
                   "soft_limit": 0.95,
                   "torque_offset": 50.0}

motor_id = [101]
impedance_params = dict()
# init_pos = {104: 0.48, 105: 0.0, 102: -0.0, 101: -0.148, 100: 0.0, 
#             103: 0.0, 110: -0.48, 111: -0.0, 108: 0.0, 107: 0.148, 106: 0.0, 109: -0.0}

init_pos = {101: 0.0}
for i in motor_id:
    impedance_params[i] = {"kp": 50.0, "kd": 5.0, "max_torque": 30} 
conf_dict = {101: watchdog_config}
# Create all candles object and add one motor
handler = pyCandle.MultipleCandles(True) 
handler.addMd80(conf_dict)
handler.setModeMd80(motor_id, "IMPEDANCE")

# set savgol
handler.setSavgol(motor_id, [34.18803419,  25.64102564,  17.09401709,   8.54700855,   0.0, -8.54700855, -17.09401709, -25.64102564, -34.18803419])
# set kalman filter
vel_var = 0.00128 #lhy - 0.00128 ; lar - 0.00128 ; lk - 4.05e-5
pos_var = 1.5e-7  #lhy - 1.5e-7 ; lar - 4.166e-9 ; 
kalman_proccess_noise = {101: [0.1*pos_var, 0.0, 0.0 , 0.1*vel_var]}
kalman_measurment_noise = {101: [pos_var, 0.0, 0.0, vel_var]}
kalman_initial_state_noise = {101: [pos_var, 0.0, 0.0, vel_var]}
handler.setKalmanFilter(kalman_proccess_noise, kalman_measurment_noise, kalman_initial_state_noise, 500)

# set pid
pid_config = {101: {"high_p_gain": 1.0 , "high_d_gain": 0.0, "high_i_gain": 0.0,"high_max_agg": 1.91, "high_limit_scale": 1.3, "agg_window": -1}}
handler.setPIDParams(pid_config)


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
    time.sleep(1)  # Add some delay


# Close the update loop
handler.disableAllMotors()

sys.exit("EXIT SUCCESS")
