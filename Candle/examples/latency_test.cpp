#include "candle.hpp"
#include <iostream>
#include <unistd.h>
#include <numeric>
#include <algorithm>
#include <string.h>



/* change the scheduler priority for this task (shouldmake the results more stable) */
#define CHANGE_PRIORITY    1
/* communication bus */
#define BUS                mab::BusType_E::SPI

/* how many tests to conduct for averaging purposes */
const int tests = 50;


const uint32_t seq = 0xdeadbeef;

int main()
{
#ifdef BENCHMARKING

#ifdef CHANGE_PRIORITY
        /* change the priority */
        struct sched_param sp;
        memset(&sp, 0, sizeof(sp));
        sp.sched_priority = 99;
        sched_setscheduler(0, SCHED_FIFO, &sp);
#endif

    //Create CANdle object and set FDCAN baudrate to 8Mbps
    mab::Candle candle(mab::CAN_BAUD_8M, false, mab::CANdleFastMode_E::FAST1, true, BUS);

    std::vector<uint16_t> ids;
    ids.push_back(200);
    ids.push_back(300);
    ids.push_back(307);
    ids.push_back(308);
    ids.push_back(309);
    ids.push_back(310);
    // ids.push_back(311);

    for(auto &id : ids)
        candle.addMd80(id);

    /* measured time delta vector */
    std::vector<long long> delta_times;

    for(int i=0;i<tests;i++)
    {
        for(auto &id : ids)
        {
            candle.controlMd80SetEncoderZero(id);       // Reset encoder at current position
            candle.controlMd80Mode(id, mab::Md80Mode_E::IMPEDANCE);     //Set mode to impedance control
            candle.controlMd80Enable(id, true);     //Enable the drive
        }
        
        candle.md80s[0].setTargetPosition(0.0f);
        candle.md80s[0].setTargetVelocity(0.0f);
        candle.benchSetFlagTx(false);
        std::cout<<"candle begin"<<std::endl;
        candle.begin();
        usleep(500);

        std::cout<<"flag deployed"<<std::endl;
        candle.benchSetFlagTx(true);
        candle.benchSetFlagRx(false);

        while(1)
        {
            candle.md80s[0].setTargetPosition(*(float*)&seq);
            candle.md80s[0].setTargetVelocity(*(float*)&seq); 
            // candle.md80s[1].setTargetPosition(0.0f);
            // candle.md80s[0].setTargetPosition(0.0f);
            usleep(5000);
            if(candle.benchGetFlagRx())
            {
                candle.md80s[0].setTargetPosition(0.0f);
                candle.md80s[0].setTargetVelocity(0.0f);
                // candle.md80s[0].setImpedanceControllerParams(0.01,0.001);
                usleep(10000);
                break;
            }  
        }
        candle.end();

        std::cout<<"candle end"<<std::endl;
        delta_times.push_back(candle.benchGetTimeDelta());
        usleep(10000);
    }

    /* cout the measured delta times vector */
    for(auto time : delta_times)
        std::cout<<time<<std::endl;

    /* calculate mean and stdev */
    double sum = std::accumulate(std::begin(delta_times), std::end(delta_times), 0.0);
    double m =  sum / delta_times.size();

    double accum = 0.0;
    std::for_each (std::begin(delta_times), std::end(delta_times), [&](const double d) {
        accum += (d - m) * (d - m);
    });

    double stdev = sqrt(accum / (delta_times.size()-1));

    double mean_f = 1e6/m;

    std::cout<<"*************** RESULTS ***************"<<std::endl;
    std::cout<<"STD DEV DELAY: "<<stdev<<std::endl;
    std::cout<<"MEAN DELAY: "<<m<<std::endl;
    std::cout<<"MAX DELAY: "<<*max_element(std::begin(delta_times), std::end(delta_times))<<std::endl;
    std::cout<<"MIN DELAY: "<<*min_element(std::begin(delta_times), std::end(delta_times))<<std::endl;
    std::cout<<"***************************************"<<std::endl;
    std::cout<<"STD DEV FREQUENCY: "<<fabs(mean_f-(1e6f/(stdev+m))) <<std::endl;
    std::cout<<"MEAN FREQUENCY: "<<mean_f<<std::endl;
    
#endif
    return EXIT_SUCCESS;
}
