import pyCandle
import random
import sys

# Create CANdle object and set FDCAN baudrate to 1Mbps
candle = pyCandle.Candle(pyCandle.CAN_BAUD_1M,True)

# Ping FDCAN bus in search of drives
ids = candle.ping()

if len(ids) == 0: # If no drives found -> quit
    sys.exit("EXIT FALIURE") 

newFDCanId = random.randint(10, 1000)   # Generate random id in range 10 - 1000

# any commands starting with config* can be accessed without adding Md80 to List (addMd80 method)
# Set motor current limit 
candle.configMd80SetCurrentLimit(ids[0], 2.5);    
# Set custom FDCAN parameters of  the drive
candle.configMd80Can(ids[0], newFDCanId, pyCandle.CAN_BAUD_1M, 250);    

# Save current limit setting and CAN configuration. Note this is commented out by default not to mess your drives
# candle.configMd80Save(newFDCanId);      

sys.exit("EXIT SUCCESS")
