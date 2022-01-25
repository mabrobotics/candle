#include "candle.hpp"

#include <iostream>
#include <unistd.h>

int main()
{
    //Create CANdle object and set FDCAN baudrate to 1Mbps
    mab::Candle candle(mab::CAN_BAUD_1M);

    //Ping FDCAN bus in search of drives
    auto ids = candle.ping();
    
    if(ids.size() == 0)
        return EXIT_FAILURE;

    //any commands starting with config* can be accessed without adding Md80 to List (addMd80 method)
    candle.configMd80SetCurrentLimit(ids[0], 2.5f);     // Set motor current limit 
    candle.configMd80Can(ids[0], 55, mab::CAN_BAUD_1M, 250);    //Set custom FDCAN parameters of  the drive
    candle.configMd80Save(ids[0]);      // Save current limit setting and CAN configuration

    candle.controlMd80SetEncoderZero(ids[0]);       // Reset the absolute encoder at current position

    return EXIT_SUCCESS;
}