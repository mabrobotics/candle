#include "candle.hpp"

#include <unistd.h>
#include <iostream>
int main()
{
    mab::Candle candle;
    if(candle.transmitConfig(100, 200, 250))
    {
        std::cout << "Got Config confirmation" << std::endl;
    }
}