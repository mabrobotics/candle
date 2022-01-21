#include "candle.hpp"

#include <unistd.h>
#include <iostream>

int main()
{
    mab::Candle candle(mab::CAN_BAUD_8M);
    if(!candle.addMd80(70))
        return 1;
    candle.controlMd80Mode(70, mab::Md80Mode_E::IMPEDANCE);
    candle.controlMd80Enable(70, true);
    candle.md80s[0].setImpedanceRegulator(1.0, 0.01);
    candle.begin();

    for(int i = 0; i < 100; i++)
    {
        std::cout << candle.md80s[0].getPosition() << std::endl;
        usleep(100000);
    }


    candle.end();
}