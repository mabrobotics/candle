#include "candle.hpp"

#include <unistd.h>
#include <iostream>

int main()
{
    mab::Candle candle(mab::CAN_BAUD_8M);

    candle.addMd80(70);
    return 1;
    candle.begin();


    candle.end();
}