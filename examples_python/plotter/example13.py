import pyCandle
import sys
import PySimpleGUI as sg
import asyncio
import time
from MotorPlotter import LivePlot

candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)
TIME_OF_TEST = 5.0 # seconds
VELOCITY_SETPOINT = 50.0 # rad/s
PLOT_UPDATE_RATE = 0.01 # seconds


sg.theme('Dark')
text = sg.Text("Motor Output")

# Define the window's contents
layout = [[text],
            [sg.Combo(['Velocity Mode', 'Position Mode'], default_value='Velocity Mode', key='-MODE-')],
            [sg.Text("Kp"),sg.InputText(0.0, key='-KP-')],
            [sg.Text("Ki"),sg.InputText(0.0, key='-KI-')],
            [sg.Text("Kd"),sg.InputText(0.0, key='-KD-')],
            [sg.Text("Windup"),sg.InputText(0.0, key='-WINDUP-')],
            [sg.Text("Max Velocity"),sg.InputText(0.0, key='-MAXVEL-')],
            [sg.Text("Max Torque"),sg.InputText(0.0, key='-MAXTORQUE-')],
            [sg.Text("Time of test [s]"),sg.Slider(range=(0, 30), orientation='h', size=(50, 10), default_value=3, key='-TIMEOFTEST-')],
            [sg.Text("Setpoint") ,sg.InputText(VELOCITY_SETPOINT, key='-SETPOINT-')],
            [sg.Button('Run',highlight_colors=('black', 'green'), button_color=('black', 'green')),
             sg.Button('Stop',highlight_colors=('black', 'red'), button_color=('black', 'red'))],
             [sg.Checkbox('Velocity [rad/s]', default=True, key='-VELOCITY-')],
             [sg.Checkbox('Position [rad]', default=False, key='-POSITION-')],
             [sg.Checkbox('Torque [Nm]', default=False, key='-TORQUE-')],
             [sg.Checkbox('Temperature [C]', default=False, key='-TEMP-')]]


# Create the window
window = sg.Window('Example13', layout)


def StopRun(runTask):
    print("Stopping program")
    candle.end()
    for md in candle.md80s:
        candle.controlMd80Enable(md, False)
    if(runTask != None):
        runTask.cancel()
    

async def EventLoop():
    runTask = None
    ids = candle.ping()
    if len(ids) == 0: # If no drives found -> quit
        sys.exit("EXIT FALIURE") 
    for id in ids:
        candle.addMd80(id)
    for md in candle.md80s:
        candle.controlMd80SetEncoderZero(md)      #  Reset encoder at current position
    
    
    event, values = window.read(timeout=0)
    reg = candle.getMd80FromList(ids[0]).getReadReg()
    window["-KP-"].update(str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorVelPidKp, reg.RW.velocityPidGains.kp)))
    window["-KI-"].update(str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorVelPidKi, reg.RW.velocityPidGains.ki)))
    window["-KD-"].update(str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorVelPidKd, reg.RW.velocityPidGains.kd)))
    window["-WINDUP-"].update(str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorVelPidWindup, reg.RW.velocityPidGains.intWindup)))
    window["-MAXTORQUE-"].update(str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorVelPidOutMax, reg.RW.velocityPidGains.outMax)))
    
    
    while True:
        await asyncio.sleep(0.2)
        event, values = window.read(timeout=0)
        if event == sg.WIN_CLOSED:
            StopRun(runTask)
            break
        if event == 'Stop':
            StopRun(runTask)
        if event == 'Run':
            if(runTask == None):
                runTask = asyncio.create_task(Run(mode=values['-MODE-'],setpoint=float(values['-SETPOINT-']), showVelocity=values['-VELOCITY-'],
                                                showPosition=values['-POSITION-'], showTorque=values['-TORQUE-'],
                                                showTemperature=values['-TEMP-'], timeOfTest=float(values['-TIMEOFTEST-']),
                                                kp=float(values['-KP-']), ki=float(values['-KI-']), kd=float(values['-KD-']),
                                                maxVel=float(values['-MAXVEL-']), maxTorque=float(values['-MAXTORQUE-']),
                                                windup=float(values['-WINDUP-'])))
            else:
                if(runTask.done()):
                    runTask = asyncio.create_task(Run(mode=values['-MODE-'],setpoint=float(values['-SETPOINT-']), showVelocity=values['-VELOCITY-'],
                                                    showPosition=values['-POSITION-'], showTorque=values['-TORQUE-'],
                                                    showTemperature=values['-TEMP-'], timeOfTest=float(values['-TIMEOFTEST-']),
                                                    kp=float(values['-KP-']), ki=float(values['-KI-']), kd=float(values['-KD-']),
                                                    maxVel=float(values['-MAXVEL-']), maxTorque=float(values['-MAXTORQUE-']),
                                                    windup=float(values['-WINDUP-'])))
    candle.end()
    candle.controlMd80Enable(ids[0], False) # Enable the drive
    window.close()

async def Run(mode='Velocity Mode', setpoint=VELOCITY_SETPOINT, showVelocity=True, showPosition=False,
               showTorque=False, showTemperature=False, timeOfTest=TIME_OF_TEST,
               kp=0.0, ki=0.0, kd=0.0, maxVel=0.0, maxTorque=0.0, windup=0.0):
    lp = LivePlot(xlim=(0,timeOfTest),ylim=(0,setpoint*1.5))

    candle.controlMd80SetEncoderZero(candle.md80s[0])

    if(mode == 'Velocity Mode'):
        candle.controlMd80Mode(candle.md80s[0], pyCandle.VELOCITY_PID)
        candle.md80s[0].setVelocityControllerParams(kp, ki, kd, windup)
        candle.md80s[0].setTargetVelocity(0.0)
    else:
        candle.controlMd80Mode(candle.md80s[0], pyCandle.POSITION_PID)
        candle.md80s[0].setTargetPosition(0.0)
        candle.md80s[0].setPositionControllerParams(kp, ki, kd, windup)

    candle.md80s[0].setMaxVelocity(maxVel)
    candle.md80s[0].setMaxTorque(maxTorque)
        
    candle.controlMd80Enable(candle.md80s[0], True)    
    candle.begin()
    if(showVelocity):
        lp.add_line("Velocity")
    if(showPosition):
        lp.add_line("Position")
    if(showTorque):
        lp.add_line("Torque")
    if(showTemperature):
        lp.add_line("Temperature")
    lp.add_line("Setpoint")
    startTime = time.time()
    trueSetpoint = 0.0
    while(time.time() - startTime < timeOfTest):
        timeStamp = time.time() - startTime
        if(showVelocity):
            lp.append_data("Velocity", timeStamp, float(candle.md80s[0].getVelocity()))
        if(showPosition):
            lp.append_data("Position", timeStamp, float(candle.md80s[0].getPosition()))
        if(showTorque):
            lp.append_data("Torque", timeStamp, float(candle.md80s[0].getTorque()))
        if(showTemperature):
            lp.append_data("Temperature", timeStamp, float(candle.md80s[0].getTemperature()))
        lp.append_data("Setpoint", timeStamp, trueSetpoint)
        if timeStamp > 0.1*timeOfTest:
            if(mode == 'Velocity Mode'):
                candle.md80s[0].setTargetVelocity(setpoint)
            else:
                candle.md80s[0].setTargetPosition(setpoint)
            trueSetpoint = setpoint
        lp.show(time=0.0001)
        await asyncio.sleep(0.0002)
    candle.md80s[0].setTargetVelocity(0.0)
    await asyncio.sleep(1.0)
    candle.end()
    candle.controlMd80Enable(candle.md80s[0], False)
    while lp.isShowing():
        lp.show(time=0.5)
        await asyncio.sleep(0.05)

    

async def main():                                
    # Display and interact with the Window
    asyncio.create_task(EventLoop())
    while True:
        await asyncio.sleep(1)
        if window.is_closed():
            break


if __name__ == "__main__":
    asyncio.run(main())

#TODO: Add boxes for showing limits
#TODO: Add option to change PID parameters