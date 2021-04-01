#include "md80.hpp"
#include "canalizator.hpp"
#include "utils/calibration.hpp"

#include <iostream>
#include "unistd.h"
#include <cmath>

int main(int argc, char **argv)
{
    Canalizator can("/dev/ttyUSB0", 460800, 1000000);
    if (!can.isOk())
        return -1;
    std::vector<Md80> drives;
    std::vector<int> foundDrives = Md80::sendPing(&can, 0x70, 0x90);
    std::cout << "foundDrives: " << foundDrives.size() << std::endl;
    for(int i = 0; i < (int)foundDrives.size(); i++)
    {
        std::cout << "[" << i << "]\tID: " << foundDrives[i] << std::endl; 
        drives.push_back(Md80(&can, foundDrives[i]));
    }
    if (foundDrives.size() < 1)
    {
        return -1;        
    }
    drives.push_back(Md80(&can, 132));
    drives.push_back(Md80(&can, 118));

    perform(&drives[0], &drives[1]);

    drives[0].enableMotor(false);
    drives[1].enableMotor(false);
}