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

    candle.controlMd80Mode(ids[0], mab::Md80Mode_E::VELOCITY_PID);     //Set mode to impedance control
    candle.controlMd80Enable(ids[0], true);     //Enable the drive

    // Now we set up Velocity Regulator, and additionaly max output velocity just to be on a safe side
    candle.md80s.at(ids[0]).setVelocityControllerParams(0.05f, 0.25f, 0.0, 1.0f);
    candle.md80s.at(ids[0]).setMaxVelocity(30.0f);

    // To reload default regulator parameters, simply disable the drive (contorlMd80Enable(id, false)), 
    // stop the communications (candle.end()) or power cycle the drive (off-on).
    
    float t = 0.0f;
    float dt = 0.04f;
    
    //Begin update loop (it starts in the background)
    candle.begin();

    float targetVelocity = 20.0f;
    for(int i = 0; i < 2000; i++)
    {
        if(i % 200 == 0)
            targetVelocity += 1.0f;
        t+=dt;
        candle.md80s.at(ids[0]).setTargetVelocity(targetVelocity);  
        std::cout << "Drive ID = " << candle.md80s.at(ids[0]).getId() << " Velocity: " << candle.md80s.at(ids[0]).getVelocity() << std::endl;
        usleep(10000);
    }
    

    //Close the update loop
    candle.end();

    return EXIT_SUCCESS;
}