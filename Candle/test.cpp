#include "candle.hpp"

#include <unistd.h>
#include <iostream>

int main()
{
    mab::Candle candle(mab::CAN_BAUD_8M);

    candle.configMd80SetCurrentLimit(70, 1.0f);
    candle.configMd80Can(70, 70, mab::CANdleBaudrate_E::CAN_BAUD_8M, 1000);
    candle.controlMd80Mode(70, mab::Md80Mode_E::IMPEDANCE);
    candle.controlMd80Enable(70, true);
    return 1;
    
    candle.begin();

    sleep(10);

    candle.end();
}