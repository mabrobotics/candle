import mab.pyCandle as pyCandle
import sys

# Create CANdle object and ping FDCAN bus in search of drives. 
# Any found drives will be printed out by the ping() method.
candles = []
while True:
    try:
        candles.append(pyCandle.Candle(pyCandle.CAN_BAUD_1M,True))
        ids = candles[-1].ping()
        print(ids)
    except:
        break


sys.exit("EXIT SUCCESS")