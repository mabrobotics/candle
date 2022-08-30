import pyCandle
import sys

# Create CANdle object and ping FDCAN bus in search of drives. 
# Any found drives will be printed out by the ping() method.

candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M,True)
ids = candle.ping()

# Blink LEDs on each drive found

for id in ids:
    candle.configMd80Blink(id)

sys.exit("EXIT SUCCESS")
