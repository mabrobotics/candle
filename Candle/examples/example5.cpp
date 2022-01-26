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

    candle.controlMd80SetEncoderZero(ids[0]);       // Reset encoder at current position

    candle.controlMd80Mode(ids[0], mab::Md80Mode_E::IMPEDANCE);     //Set mode to impedance control
    candle.controlMd80Enable(ids[0], true);     //Enable the drive

    // Now we modify the Impedance regulator parameters - the drive will behave much different than in 
    // previous examples. The drive will change default params to the ones we select below.
    candle.md80s[0].setImpedanceRegulator(5.0, 0.5);

    // To reload default regulator parameters, simply disable the drive (contorlMd80Enable(id, false)), 
    // stop the communications (candle.end()) or power cycle the drive (off-on).
    
    float t = 0.0f;
    float dt = 0.04f;
    
    //Begin update loop (it starts in the background)
    candle.begin();

    for(int i = 0; i < 1000; i++)
    {
        t+=dt;
        candle.md80s[0].setTargetPosition(sin(t));  
        usleep(10000);
    }

    //Close the update loop
    candle.end();

    return EXIT_SUCCESS;
}