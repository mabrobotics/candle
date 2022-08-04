#include "candle.hpp"
#include <iostream>
#include <unistd.h>
#include <numeric>
#include <algorithm>
#include <string.h>

/* change the scheduler priority for this task (shouldmake the results more stable) */
const bool changePriority = true;
/* communication bus */
mab::BusType_E bus = mab::BusType_E::SPI;
/* communication speed mode */
mab::CANdleFastMode_E mode = mab::CANdleFastMode_E::FAST1;
/* how many tests to conduct for averaging purposes */
const int tests = 50;

/* sequence that triggers error bit EMPTY7 */
const uint32_t seq = 0xdeadbeef;

void wrongArguments()
{
	std::cout << "Wrong arguments specified, please see ./latency_test --help"<<std::endl;
	exit(-1);
}

int main(int argc, char *argv[])
{

    if(argc < 2)wrongArguments();

    if(strcmp(argv[1],"--help") == 0)
	{
		std::cout << "usage: ./latency_test <bus> <mode> [--help]" << std::endl;
		std::cout << "<bus> can be SPI/USB/UART" << std::endl;
		std::cout << "<mode> can be NORMAL/FAST1/FAST2" << std::endl;
		std::cout << "[--help] - displays help message" << std::endl;
		return -1;
	}

    if(strcmp(argv[1],"SPI") == 0)bus = mab::BusType_E::SPI;
    else if(strcmp(argv[1],"USB") == 0)bus = mab::BusType_E::USB;
    else if(strcmp(argv[1],"UART") == 0)bus = mab::BusType_E::UART;
    else
    {
        std::cout<<"bus parameter not recognised!"<<std::endl;
        return -1;
    }

    if(strcmp(argv[2],"NORMAL") == 0)mode = mab::CANdleFastMode_E::NORMAL;
    else if(strcmp(argv[2],"FAST1") == 0)mode = mab::CANdleFastMode_E::FAST1;
    else if(strcmp(argv[2],"FAST2") == 0)mode = mab::CANdleFastMode_E::FAST2;
    else
    {
        std::cout<<"mode parameter not recognised!"<<std::endl;
        return -1;
    }
    
#ifdef BENCHMARKING

    if(changePriority)
    {
        /* change the priority */
        struct sched_param sp;
        memset(&sp, 0, sizeof(sp));
        sp.sched_priority = 99;
        sched_setscheduler(0, SCHED_FIFO, &sp);
    }

    /* Create CANdle object and set FDCAN baudrate to 8Mbps */
    mab::Candle candle(mab::CAN_BAUD_8M, false, mode, true, bus);

    std::vector<uint16_t> ids;

    switch(mode)
    {
        case mab::CANdleFastMode_E::FAST2:
        {
            ids.push_back(200);
            ids.push_back(301);
            ids.push_back(302);
            break;
        }
        case mab::CANdleFastMode_E::FAST1:
        {
            ids.push_back(200);
            ids.push_back(301);
            ids.push_back(302);
            ids.push_back(303);
            ids.push_back(304);
            ids.push_back(305);
            break;
        }
        case mab::CANdleFastMode_E::NORMAL:
        {
            ids.push_back(200);
            ids.push_back(301);
            ids.push_back(302);
            ids.push_back(303);
            ids.push_back(304);
            ids.push_back(305);
            ids.push_back(306);
            ids.push_back(307);
            ids.push_back(308);
            ids.push_back(309);
            ids.push_back(310);
            ids.push_back(311);
            break;
        }
    }

    for(auto &id : ids)
        candle.addMd80(id);

    /* measured time delta vector */
    std::vector<long long> deltaTimes;

    for(int i=0;i<tests;i++)
    {
        for(auto &id : ids)
        {
            candle.controlMd80SetEncoderZero(id);       // Reset encoder at current position
            candle.controlMd80Mode(id, mab::Md80Mode_E::IMPEDANCE);     //Set mode to impedance control
            candle.controlMd80Enable(id, true);     //Enable the drive
        }
        usleep(5000);
        candle.md80s[0].setTargetPosition(0.0f);
        candle.md80s[0].setTargetVelocity(0.0f);
        candle.benchSetFlagTx(false);
        std::cout<<"candle begin"<<std::endl;
        candle.begin();
        usleep(5000);

        std::cout<<"flag deployed"<<std::endl;
        candle.benchSetFlagTx(true);
        candle.benchSetFlagRx(false);

        while(1)
        {
            candle.md80s[0].setTargetPosition(*(float*)&seq);
            candle.md80s[0].setTargetVelocity(*(float*)&seq); 
            usleep(5000);
            if(candle.benchGetFlagRx())
            {
                candle.md80s[0].setTargetPosition(0.0f);
                candle.md80s[0].setTargetVelocity(0.0f);
                usleep(10000);
                break;
            }  
        }
        candle.end();

        std::cout<<"candle end"<<std::endl;
        deltaTimes.push_back(candle.benchGetTimeDelta());
        usleep(10000);
    }

    /* cout the measured delta times vector */
    for(auto time : deltaTimes)
        std::cout<<time<<std::endl;

    /* calculate mean and stdev */
    double sum = std::accumulate(std::begin(deltaTimes), std::end(deltaTimes), 0.0);
    double m =  sum / deltaTimes.size();

    double accum = 0.0;
    std::for_each (std::begin(deltaTimes), std::end(deltaTimes), [&](const double d) {
        accum += (d - m) * (d - m);
    });

    double stdev = sqrt(accum / (deltaTimes.size()-1));
    double meanFreq = 1e6/m;
    double stdevFreq = fabs(meanFreq-(1e6f/(stdev+m)));
    double minDelay = *min_element(std::begin(deltaTimes), std::end(deltaTimes));
    double maxDelay = *max_element(std::begin(deltaTimes), std::end(deltaTimes));

    std::cout<<"*************** RESULTS ***************"<<std::endl;
    std::cout<<"MEAN DELAY: "<<m<<" us"<<std::endl;
    std::cout<<"STD DEV DELAY: "<<stdev<<" us"<<std::endl;
    std::cout<<"MIN DELAY: "<<minDelay<<" us"<<std::endl;
    std::cout<<"MAX DELAY: "<<maxDelay<<" us"<<std::endl;
    std::cout<<"MEAN FREQUENCY: "<<meanFreq<<" Hz"<<std::endl;
    std::cout<<"STD DEV FREQUENCY: "<<stdevFreq<<" Hz"<<std::endl<<std::endl;
    std::cout<<"*** COPY & PASTE TO GOOGLE SHEETS *****"<<std::endl;
    std::cout<<m<<";"<<stdev<<";"<<minDelay<<";"<<maxDelay<<";"<<meanFreq<<";"<<stdevFreq<<std::endl;
    
#endif
    return EXIT_SUCCESS;
}
