#include "candle.hpp"

#include <unistd.h>
#include <iostream>

int main()
{
    mab::Candle candle;

    candle.ping();

    // sleep(5);
    // candle.configMd80Can(70, 75, mab::CANdleBaudrate_E::CAN_BAUD_8M, 100);
    return 1;
    
    candle.begin();

    sleep(10);

    candle.end();
}