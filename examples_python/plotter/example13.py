################################################################################
#
# This example shows how to run a motor and plot the data in real time.
#
#
# Requirements:
# - pyCandle
# - matplotlib
# - numpy
# - PySimpleGUI
#

################################################################################
# Imports
################################################################################
import pyCandle
import sys
import PySimpleGUI as sg
import asyncio
import time
from MotorPlotter import LivePlot

################################################################################
# Constants
################################################################################

TIME_OF_TEST = 5.0  # seconds
VELOCITY_SETPOINT = 50.0  # rad/s
PLOT_UPDATE_RATE = 0.01  # seconds
sg.theme('Dark')

################################################################################
# Functions
################################################################################

# Define the window's contents
layout = [[sg.Text("Motor Output")],
          [sg.Combo(['Velocity Mode', 'Position Mode'],
                    default_value='Velocity Mode', key='-MODE-', enable_events=True)],
          [sg.Text("Kp"), sg.InputText(0.0, key='-KP-')],
          [sg.Text("Ki"), sg.InputText(0.0, key='-KI-')],
          [sg.Text("Kd"), sg.InputText(0.0, key='-KD-')],
          [sg.Text("Windup"), sg.InputText(0.0, key='-WINDUP-')],
          [sg.Text("Max Velocity"), sg.InputText(0.0, key='-MAXVEL-')],
          [sg.Text("Max Torque"), sg.InputText(0.0, key='-MAXTORQUE-')],
          [sg.Text("Time of test [s]"), sg.Slider(range=(0, 30), orientation='h', size=(50, 10), default_value=3, key='-TIMEOFTEST-')],
          [sg.Text("Setpoint"), sg.InputText(VELOCITY_SETPOINT, key='-SETPOINT-')],
          [sg.Button('Run', highlight_colors=('black', 'green'), button_color=('black', 'green')),
           sg.Button('Stop', highlight_colors=('black', 'red'), button_color=('black', 'red')),
           sg.Button('Save', highlight_colors=('black', 'grey'), button_color=('black', 'grey'))],
          [sg.Checkbox('Velocity [rad/s]', default=True, key='-VELOCITY-')],
          [sg.Checkbox('Position [rad]', default=False, key='-POSITION-')],
          [sg.Checkbox('Torque [Nm]', default=False, key='-TORQUE-')],
          [sg.Checkbox('Temperature [C]', default=False, key='-TEMP-')]]


# Create the window
window = sg.Window('Example13', layout)

# Create global candle object
candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)


def StopRun(runTask, taskFunction):
    """
    Disables the motor and cancels the task
    runTask: asyncio task
    """
    print("Stopping program")
    candle.end()
    for md in candle.md80s:
        candle.controlMd80Enable(md, False)

    if (taskFunction != None):
        taskFunction.lp.close() # Close the window of the live plot

    if (runTask != None):
        try:
            runTask.cancel()
        finally:
            print("Task cancelled")

def SaveCurrentConfig(id,mode,kp,ki,kd,windup,maxTorque,maxVel):
    """
    Save current PID settings in the drive
    params:
    mode: string, "Velocity Mode" or "Position Mode"
    kp: float, proportional gain
    ki: float, integral gain
    kd: float, derivative gain
    windup: float, windup limit
    outMax: float, output limit
    id: int, id of the drive
    """
    print("Saving current configuration")
    print("Mode: " + mode)
    print("Kp: " + str(kp))
    print("Ki: " + str(ki))
    print("Kd: " + str(kd))
    print("Windup: " + str(windup))
    print("Max Torque: " + str(maxTorque))
    print("Max Velocity: " + str(maxVel))
    if mode == "Velocity Mode":
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorVelPidKp,float(kp))
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorVelPidKi,float(ki))
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorVelPidKd,float(kd))
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorVelPidWindup,float(windup))
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorVelPidOutMax,float(maxTorque))
    elif mode == "Position Mode":
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorPosPidKp,float(kp))
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorPosPidKi,float(ki))
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorPosPidKd,float(kd))
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorPosPidWindup,float(windup))
        candle.writeMd80Register(id,pyCandle.Md80Reg_E.motorPosPidOutMax,float(maxVel))
    
    candle.configMd80Save(id) # Save the configuration in the drive


async def EventLoop():
    """
    Event loop for the GUI, starts run command when button is pressed
    """
    runTask = None
    taskFunction = None
    ids = candle.ping()  # Find all connected drives
    if len(ids) == 0:  # If no drives found -> quit
        sys.exit("EXIT FALIURE")
    for id in ids:
        candle.addMd80(id)  # Add all drives to the candle object

    event, values = window.read(timeout=0)  # Initialise GUI
    reg = candle.getMd80FromList(ids[0]).getReadReg()  # Get register object

    ###### Set initial values of PID registers (velocity by default) ######
    window["-KP-"].update(str(candle.readMd80Register(ids[0],
                          pyCandle.Md80Reg_E.motorVelPidKp, reg.RW.velocityPidGains.kp)))
    window["-KI-"].update(str(candle.readMd80Register(ids[0],
                          pyCandle.Md80Reg_E.motorVelPidKi, reg.RW.velocityPidGains.ki)))
    window["-KD-"].update(str(candle.readMd80Register(ids[0],
                          pyCandle.Md80Reg_E.motorVelPidKd, reg.RW.velocityPidGains.kd)))
    window["-WINDUP-"].update(str(candle.readMd80Register(
        ids[0], pyCandle.Md80Reg_E.motorVelPidWindup, reg.RW.velocityPidGains.intWindup)))
    window["-MAXTORQUE-"].update(str(candle.readMd80Register(
        ids[0], pyCandle.Md80Reg_E.motorVelPidOutMax, reg.RW.velocityPidGains.outMax)))

    # Run the GUI event loop
    while True:
        await asyncio.sleep(0.2)
        event, values = window.read(timeout=0)
        if event == sg.WIN_CLOSED:  # If user closes window or clicks cancel
            StopRun(runTask, taskFunction)
            break
        if event == 'Stop':  # If user clicks stop, disable the motor
            StopRun(runTask, taskFunction)
        if event == 'Run':  # If user clicks run, start the test

            taskFunction = Run  # Asign the task function

            if (runTask == None):  # First time running
                runTask = asyncio.create_task(taskFunction(mode=values['-MODE-'], setpoint=float(values['-SETPOINT-']), showVelocity=values['-VELOCITY-'],
                                                           showPosition=values['-POSITION-'], showTorque=values['-TORQUE-'],
                                                           showTemperature=values['-TEMP-'], timeOfTest=float(
                                                               values['-TIMEOFTEST-']),
                                                           kp=float(values['-KP-']), ki=float(values['-KI-']), kd=float(values['-KD-']),
                                                           maxVel=float(values['-MAXVEL-']), maxTorque=float(values['-MAXTORQUE-']),
                                                           windup=float(values['-WINDUP-'])))
            else:
                if (runTask.done()):  # Another time running, check for previous task to be done
                    runTask = asyncio.create_task(taskFunction(mode=values['-MODE-'], setpoint=float(values['-SETPOINT-']), showVelocity=values['-VELOCITY-'],
                                                               showPosition=values['-POSITION-'], showTorque=values['-TORQUE-'],
                                                               showTemperature=values['-TEMP-'], timeOfTest=float(
                                                                   values['-TIMEOFTEST-']),
                                                               kp=float(values['-KP-']), ki=float(values['-KI-']), kd=float(values['-KD-']),
                                                               maxVel=float(values['-MAXVEL-']), maxTorque=float(values['-MAXTORQUE-']),
                                                               windup=float(values['-WINDUP-'])))
                else:  # If previous task is not done, cancel it
                    StopRun(runTask, taskFunction)
        if event == 'Save':  # If user clicks save, save the data
            SaveCurrentConfig(ids[0],values["-MODE-"],values["-KP-"],
                              values["-KI-"],values["-KD-"],values["-WINDUP-"],
                              values["-MAXTORQUE-"],values["-MAXVEL-"])
            # After update of the config, wait for the drive to reboot
            while(True):
                ids = candle.ping() 
                if len(ids) != 0:
                    for id in ids:
                        candle.addMd80(id)  # Add all drives to the candle object
                    break
                await asyncio.sleep(1.0)


        if event == '-MODE-':  # If user changes mode, load PID registers for apropriate PID controller
            print("Mode changed to: " + values['-MODE-'])
            reg = candle.getMd80FromList(ids[0]).getReadReg()
            if values['-MODE-'] == 'Velocity Mode':
                window['-KP-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorVelPidKp, reg.RW.velocityPidGains.kp)))
                window['-KI-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorVelPidKi, reg.RW.velocityPidGains.ki)))
                window['-KD-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorVelPidKd, reg.RW.velocityPidGains.kd)))
                window['-WINDUP-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorVelPidWindup, reg.RW.velocityPidGains.intWindup)))
                window['-MAXTORQUE-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorVelPidOutMax, reg.RW.velocityPidGains.outMax)))
            if values['-MODE-'] == 'Position Mode':
                window['-KP-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorPosPidKp, reg.RW.positionPidGains.kp)))
                window['-KI-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorPosPidKi, reg.RW.positionPidGains.ki)))
                window['-KD-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorPosPidKd, reg.RW.positionPidGains.kd)))
                window['-WINDUP-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorPosPidWindup, reg.RW.positionPidGains.intWindup)))
                window['-MAXTORQUE-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorVelPidOutMax, reg.RW.velocityPidGains.outMax)))
                window['-MAXVEL-'].update(str(candle.readMd80Register(
                    ids[0], pyCandle.Md80Reg_E.motorPosPidOutMax, reg.RW.positionPidGains.outMax)))
    candle.end()  # Close connection to candle
    candle.controlMd80Enable(ids[0], False)  # Disable motor
    window.close()


async def Run(mode='Velocity Mode', setpoint=VELOCITY_SETPOINT, showVelocity=True, showPosition=False,
              showTorque=False, showTemperature=False, timeOfTest=TIME_OF_TEST,
              kp=0.0, ki=0.0, kd=0.0, maxVel=0.0, maxTorque=0.0, windup=0.0):
    """
    Run a test with the selected parameters and display it live on a plot
    mode: 'Velocity Mode' or 'Position Mode'
    setpoint: Setpoint for the test (Velocity in rad/s or Position in rad)
    showVelocity: Show velocity on plot
    showPosition: Show position on plot
    showTorque: Show torque on plot
    showTemperature: Show temperature on plot
    timeOfTest: Time of the test in seconds
    kp: Proportional gain for PID controller
    ki: Integral gain for PID controller
    kd: Derivative gain for PID controller
    maxVel: Maximum velocity for the motor (only for position mode)
    maxTorque: Maximum torque for the motor
    windup: Windup for the PID controller
    """

    # Create a live plot with set dimensions (nessecary for live plotting)
    Run.lp = LivePlot(xlim=(0, timeOfTest), ylim=(0, setpoint*1.5))

    candle.controlMd80SetEncoderZero(candle.md80s[0])  # Set encoder zero

    # Set temporary PID parameters for apropriate PID controller
    if (mode == 'Velocity Mode'):
        candle.controlMd80Mode(candle.md80s[0], pyCandle.VELOCITY_PID)
        candle.md80s[0].setVelocityControllerParams(kp, ki, kd, windup)
        candle.md80s[0].setTargetVelocity(0.0)
    else:
        candle.controlMd80Mode(candle.md80s[0], pyCandle.POSITION_PID)
        candle.md80s[0].setTargetPosition(0.0)
        candle.md80s[0].setPositionControllerParams(kp, ki, kd, windup)

    candle.md80s[0].setMaxVelocity(maxVel)
    candle.md80s[0].setMaxTorque(maxTorque)

    candle.controlMd80Enable(candle.md80s[0], True)  # Enable motor
    candle.begin()  # Open connection to candle
    # Add chosen lines to plot
    if (showVelocity):
        Run.lp.add_line("Velocity")
    if (showPosition):
        Run.lp.add_line("Position")
    if (showTorque):
        Run.lp.add_line("Torque")
    if (showTemperature):
        Run.lp.add_line("Temperature")
    Run.lp.add_line("Setpoint")

    startTime = time.time()  # Start time of test
    trueSetpoint = 0.0  # Live setpoint for plot

    # Start test
    while (time.time() - startTime < timeOfTest):

        timeStamp = time.time() - startTime  # Time since start of test

        # Append data to plot
        if (showVelocity):
            Run.lp.append_data("Velocity", timeStamp, float(
                candle.md80s[0].getVelocity()))
        if (showPosition):
            Run.lp.append_data("Position", timeStamp, float(
                candle.md80s[0].getPosition()))
        if (showTorque):
            Run.lp.append_data("Torque", timeStamp, float(
                candle.md80s[0].getTorque()))
        if (showTemperature):
            Run.lp.append_data("Temperature", timeStamp, float(
                candle.md80s[0].getTemperature()))
        Run.lp.append_data("Setpoint", timeStamp, trueSetpoint)

        # Set setpoint at appropriate time
        if timeStamp > 0.1*timeOfTest:
            if (mode == 'Velocity Mode'):
                candle.md80s[0].setTargetVelocity(setpoint)
            else:
                candle.md80s[0].setTargetPosition(setpoint)
            trueSetpoint = setpoint

        Run.lp.show(time=0.0001)  # Show plot using plot (pause for x seconds)
        await asyncio.sleep(0.0002)  # Pause for x seconds so GUI can update

    # End test
    if (mode == 'Velocity Mode'):
        candle.md80s[0].setTargetVelocity(0.0)

    await asyncio.sleep(1.0)  # Pause for x seconds to let motor stop
    candle.end()  # Close connection to candle
    candle.controlMd80Enable(candle.md80s[0], False)  # Disable motor
    while Run.lp.isShowing():  # Wait for plot to close
        Run.lp.show(time=0.05)
        await asyncio.sleep(0.05)


################################################################################################
# Main
################################################################################################
async def main():
    asyncio.create_task(EventLoop())
    while True:
        await asyncio.sleep(1)
        if window.is_closed():
            break

# Entry point
if __name__ == "__main__":
    asyncio.run(main())
