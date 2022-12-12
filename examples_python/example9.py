import pyCandle
import time
import math
import sys  

# Create CANdle object and ping FDCAN bus in search of drives. 
# Any found drives will be printed out by the ping() method.
candle1 = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)
candle2 = pyCandle.Candle(pyCandle.CAN_BAUD_1M, True)

print("Candle 1 ID is: " + str(candle1.getUsbDeviceId()))
print("Candle 2 ID is: " + str(candle2.getUsbDeviceId()))

for id in candle1.ping(pyCandle.CAN_BAUD_1M):
    candle1.addMd80(id)

for id in candle2.ping(pyCandle.CAN_BAUD_1M):
    candle2.addMd80(id)

sys.exit("EXIT SUCCESS") 
