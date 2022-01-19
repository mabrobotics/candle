#include "candle.hpp"

#include <unistd.h>
#include <iostream>

int main()
{
    mab::Candle candle;

    std::cout << "sedning ping 1" << std::endl;
    candle.ping(1);

    std::cout << "sedning ping 8" << std::endl;
    candle.ping(8);
    return 1;
    
    candle.begin();

    sleep(10);

    candle.end();
}