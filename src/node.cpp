#include "canalizator.hpp"
#include "md80.hpp"

#include <iostream>
#include "unistd.h"
#include <cmath>

int main(int argc, char **argv)
{
    Canalizator can("/dev/ttyUSB0", 460800, 1000000);
    if (!can.isOk())
        return -1;

    Md80::initalize(&can);
    Md80 x(0x110);
    // x._calibrate();

    x.enableMotor(true);
    x.setMode(md80_mode::IMPEDANCE_CTL);
    x.setImpedance(0.1, 0.05, 0.0, 0.0, 0.0, 0.1f);
    sleep(10);
    x.enableMotor(false);
}