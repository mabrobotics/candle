import pyCandle
import sys

# Create CANdle object and ping FDCAN bus in search of drives. 
# Any found drives will be printed out by the ping() method.

candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M,True)
ids = candle.ping()

candle.addMd80(ids[0])

reg = candle.getMd80FromList(ids[0]).getReadReg()
print("")

# Lets first read some registers NOTE: registers cannot be accessed after candle.begin();
print("Drive ID: " + str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.canId, reg.RW.canId)))
print("Motor name: " + candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorName, ""))
print("Impedance mode Kp gain : " + "{:.2f}".format(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorImpPidKp, reg.RW.impedancePdGains.kp)) + " Nm/rad")
print("Impedance mode Kd gain : " + "{:.2f}".format(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorImpPidKd, reg.RW.impedancePdGains.kd)) + " Nm*s/rad")

print("")
# Then change some parameters 
candle.writeMd80Register(ids[0],pyCandle.Md80Reg_E.motorName, "EXAMPLE12")
candle.writeMd80Register(ids[0],pyCandle.Md80Reg_E.motorImpPidKp, 1.0)
candle.writeMd80Register(ids[0],pyCandle.Md80Reg_E.motorImpPidKd, 0.01)
candle.writeMd80Register(ids[0],pyCandle.Md80Reg_E.motorImpPidOutMax, 1.0)

# And read one more time NOTE: these settings are not saved!
print("Drive ID: " + str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.canId, reg.RW.canId)))
print("Motor name: " + candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorName, ""))
print("Impedance mode Kp gain : " + "{:.2f}".format(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorImpPidKp, reg.RW.impedancePdGains.kp)) + " Nm/rad")
print("Impedance mode Kd gain : " + "{:.2f}".format(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorImpPidKd, reg.RW.impedancePdGains.kd)) + " Nm*s/rad")

sys.exit("EXIT SUCCESS")
