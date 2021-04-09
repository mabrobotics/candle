#include "utils/calibration.hpp"

#include "iostream"
#include "unistd.h"
#include "cmath"

float velocityTest(Md80*tested, float torqueCmd)
{
    tested->setImpedance(0.0, 0.0, 0.0, 0.0, torqueCmd, 10.0);
    float averageVel = 0.0f;
    int samples = 75;
    for(int i = 0 ; i < samples; i++)
    {
        tested->setImpedance();
        averageVel += tested->getVelocity();
        usleep(1);
    }
    return averageVel / samples;
}
void checkCw(Md80*tested, int points, float vel[], float torque) 
{
    //Change offset to lowest value
    for(int i = 0; i<points; i++)
        tested->_changeOffsetMinus();

    //Check from lowest to highest
    for(int i = 0; i<points*2; i++)
    {
        vel[i] = velocityTest(tested, torque);
        tested->_changeOffsetPlus();
    }
    //Return to base offset
    for(int i = 0; i<points; i++)
        tested->_changeOffsetMinus();
}

void perform(Md80*tested, Md80*brake)
{
    std::cout << "Tested drive: 0x" << std::hex << tested->getId() << std::dec << std::endl;
    std::cout << "Brake  drive: 0x" << std::hex << brake->getId() << std::dec << std::endl;

    brake->setImpedance(0.0, 0.0005f, 0.0, 0.0, 0.0, 1.5);
    brake->setMode(md80_mode::IMPEDANCE_CTL);
    brake->enableMotor(true);

    tested->setImpedance(0.0, 0.0, 0.0, 0.0, 0.0, 0.5);
    tested->setMode(md80_mode::IMPEDANCE_CTL);
    tested->enableMotor(true);

    tested->setCurrentLimit(40.0f);
    brake->setCurrentLimit(20.0f);

    constexpr int points = 10;
    float offset_offset_cw[points*2] = {0.0f};
    float offset_offset_ccw[points*2] = {0.0f};

    std::cout << "Starting Rotation..." << std::endl;
    tested->setImpedance(0.0, 0.0, 0.0, 0.0, 0.0, 1.2);
    sleep(1);
    std::cout << "Starting Measurements..." << std::endl;
    checkCw(tested, points, offset_offset_cw, 1.2);
    tested->setImpedance(0.0, 0.0, 0.0, 0.0, 0.0, 1.2);
    sleep(1);
    checkCw(tested, points, offset_offset_ccw, -1.2);

    float sum[points*2] = {0.0f};
    for(int i = 0; i < points*2; i++)
        sum[i] = offset_offset_cw[i] + fabsf(offset_offset_ccw[i]);
    int bestOffset = 0;
    float highestValue = 0.0f;
    for(int i = 0; i < points*2; i++)
        if (sum[i] > highestValue)
        {
            highestValue = sum[i];
            bestOffset = i;
        }

    for(int i = 0; i < points * 2; i++)
        std::cout << i << " - CW: " << offset_offset_cw[i] << ", CCW: " << offset_offset_ccw[i] << std::endl;
    std::cout << "Best Offset: " << bestOffset << std::endl;
    if(bestOffset > points)
    {
        bestOffset -=points;
        std::cout<< "Moving offset forward(+): " << bestOffset << std::endl;
        for(int i = 0; i < bestOffset;i++)
            tested->_changeOffsetPlus();
    }
    else
    {
        std::cout<< "Moving offset back(-): " << points - bestOffset << std::endl;
        for(int i = 0; i < points - bestOffset; i++)
            tested->_changeOffsetMinus();
    }
    tested->setImpedance(0.0, 0.0005f, 0.0,0.0,0.0,0.5);
    sleep(1);

    tested->setNewConfig();
}

void aplituteTest(Md80*tested)
{

}