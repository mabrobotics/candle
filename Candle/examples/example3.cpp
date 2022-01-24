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

    candle.configMd80SetZero(ids[0]);
    candle.configMd80SetCurrentLimit(ids[0], 2.5f);
    candle.configMd80Save(ids[0]);

    return EXIT_SUCCESS;
}