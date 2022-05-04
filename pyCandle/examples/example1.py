import mab.pyCandle as pyCandle
import sys

candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M,True)
ids = candle.ping()

for id in ids:
    candle.configMd80Blink(id)

sys.exit("EXIT SUCCESS")