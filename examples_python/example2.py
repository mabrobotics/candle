import pyCandle
import time
import sys

# Create CANdle object and set FDCAN baudrate to 1Mbps
candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M,True)

# Ping FDCAN bus in search of drives
ids = candle.ping()

# Add all found to the update list
for id in ids:
    candle.addMd80(id)

# Begin update loop (it starts in the background)
candle.begin()

# Auto update loop is running in the background updating data in candle.md80s vector. Each md80 object can be 
# Called for data at any time
for i in range(1000):
    print("Drive Id: " + str(candle.md80s[0].getId()) + " Position: " + str(candle.md80s[0].getPosition()))
    time.sleep(0.1)

# Close the update loop
candle.end()

sys.exit("EXIT SUCCESS")
