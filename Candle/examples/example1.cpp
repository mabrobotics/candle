#include "candle.hpp"
#include <iostream>
#include <unistd.h>

int main()
{
    //Create CANdle object and set FDCAN baudrate to 1Mbps
    mab::Candle candle(mab::CAN_BAUD_1M, true);

    //Ping FDCAN bus in search of drives
    auto ids = candle.ping();

    if(ids.size() == 0) //If no drives found -> quit
        return EXIT_FAILURE;

    //Add all found to the update list
    for(auto &id : ids)
        candle.addMd80(id);

    uint16_t error = candle.md80s.at(ids[0]).getErrorVector();

    std::cout<<"ERROR VECTOR: 0x"<<std::hex<<error<<std::endl;

    return EXIT_SUCCESS;
}
