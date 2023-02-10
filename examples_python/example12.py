import pyCandle
import sys

# Create CANdle object and ping FDCAN bus in search of drives. 
# Any found drives will be printed out by the ping() method.

candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M,True)
ids = candle.ping()

# Blink LEDs on each drive found

candle.addMd80(ids[0])

regR = candle.getMd80FromList(ids[0]).getReadReg()

print("Motor d-axis resistance: " +str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorResistance, regR.RO.resistance))+" Ohm")
print("Motor d-axis resistance: " +str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorInductance, regR.RO.inductance))+" H")

# motor_name = "TEST"
# print(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorResistance, regR.RO.resistance))
# print(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.canTermination, regR.RW.canTermination))
# print(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorName, motor_name))

# print("was " + str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorKt_a, regR.RW.motorKt_a)))
# candle.writeMd80Register(ids[0],pyCandle.Md80Reg_E.motorKt_a, 0.666)
# print("is " + str(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorKt_a, regR.RW.motorKt_a)))

# candle.writeMd80Register(ids[0],pyCandle.Md80Reg_E.motorName, "TEST")
# print(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.motorName, motor_name))

# candle.writeMd80Register(ids[0],pyCandle.Md80Reg_E.canTermination, regR.RW.canTermination)
# print(candle.readMd80Register(ids[0],pyCandle.Md80Reg_E.canTermination, regR.RW.canTermination))

sys.exit("EXIT SUCCESS")
