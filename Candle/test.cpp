#include "candle.hpp"

#include <unistd.h>
#include <iostream>

int main()
{
    mab::Candle candle(mab::CAN_BAUD_8M);

    candle.configMd80SetCurrentLimit(70, 10.0f);
    candle.configMd80SetZero(70);
    // sleep(5);
    // candle.configMd80Can(70, 75, mab::CANdleBaudrate_E::CAN_BAUD_8M, 100);
    return 1;
    
    candle.begin();

    sleep(10);

    candle.end();
}