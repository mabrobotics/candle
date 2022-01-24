#include "candle.hpp"

#include <unistd.h>
#include <iostream>

int main()
{
    mab::Candle candle(mab::CAN_BAUD_1M);

    candle.configMd80Can(70, 70, mab::CAN_BAUD_1M, 100);
    candle.configMd80Can(71, 71, mab::CAN_BAUD_1M, 100);
    candle.configMd80Can(72, 72, mab::CAN_BAUD_1M, 100);
    candle.configMd80Can(73, 73, mab::CAN_BAUD_1M, 100);

    if(!candle.addMd80(70))
        return 1;
    if(!candle.addMd80(72))
        return 1;
    if(!candle.addMd80(73))
        return 1;
    candle.controlMd80Mode(70, mab::Md80Mode_E::IMPEDANCE);
    // candle.controlMd80Mode(71, mab::Md80Mode_E::IMPEDANCE);
    candle.controlMd80Mode(72, mab::Md80Mode_E::IMPEDANCE);
    candle.controlMd80Mode(73, mab::Md80Mode_E::IMPEDANCE);

    candle.controlMd80Enable(70, true);
    candle.controlMd80Enable(72, true);
    candle.controlMd80Enable(73, true);

    candle.md80s[0].setImpedanceRegulator(0.3, 0.01);
    candle.md80s[1].setImpedanceRegulator(0.3, 0.01);
    candle.md80s[2].setImpedanceRegulator(0.3, 0.01);
    candle.md80s[3].setImpedanceRegulator(0.3, 0.01);

    candle.begin();
    for(int i = 0; i < 1000; i++)
    {
        std::cout << candle.md80s[3].getPosition() << std::endl;
        usleep(100000);
    }
    candle.end();
}