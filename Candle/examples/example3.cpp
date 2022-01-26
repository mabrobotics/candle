#include "candle.hpp"

#include <iostream>
#include <unistd.h>

int main()
{
    //Create CANdle object and set FDCAN baudrate to 1Mbps
    mab::Candle candle(mab::CAN_BAUD_1M, true);

    //Ping FDCAN bus in search of drives
    auto ids = candle.ping();
    
    if(ids.size() == 0)
        return EXIT_FAILURE;

    uint16_t newFDCanId = 59;

    //any commands starting with config* can be accessed without adding Md80 to List (addMd80 method)
    candle.configMd80SetCurrentLimit(ids[0], 2.5f);     // Set motor current limit 
    candle.configMd80Can(ids[0], newFDCanId, mab::CAN_BAUD_1M, 250);    //Set custom FDCAN parameters of  the drive
    candle.configMd80Save(newFDCanId);      // Save current limit setting and CAN configuration

    return EXIT_SUCCESS;
}