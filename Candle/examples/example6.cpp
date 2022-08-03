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

    //Now we shall loop over all found drives to change control mode and enable them one by one
    for(auto &[motorId, md] : candle.md80s)
    {
        candle.controlMd80SetEncoderZero(md);       // Reset encoder at current position
        candle.controlMd80Mode(md, mab::Md80Mode_E::IMPEDANCE);     //Set mode to impedance control
        candle.controlMd80Enable(md, true);     //Enable the drive
    }

    
    float t = 0.0f;
    float dt = 0.04f;
    
    //Begin update loop (it starts in the background)
    candle.begin();

    for(int i = 0; i < 1000; i++)
    {
        //Once again we loop over all drives, this time setting thier position target. All drives should now perform
        //a nice synchronized movement.
        for(auto &[motorId, md] : candle.md80s)
            md.setTargetPosition(sin(t) * 2.0f);

        t+=dt;
        usleep(10000);
    }

    //Close the update loop
    candle.end();

    return EXIT_SUCCESS;
}