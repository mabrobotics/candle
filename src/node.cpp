#include "md80.hpp"
#include "canalizator.hpp"
#include "utils/calibration.hpp"

#include <iostream>
#include "unistd.h"
#include <cmath>

void findMd80(Canalizator *can, int min, int max);

int main(int argc, char **argv)
{
    Canalizator can("/dev/ttyUSB0", 460800, 1000000);
    if (!can.isOk())
        return -1;

    findMd80(&can, 0x70, 0x90);
}

void findMd80(Canalizator *can, int min, int max)
{
    
    std::vector<int> foundDrives = Md80::sendPing(can, 0x70, 0x90);
    std::cout << "foundDrives: " << foundDrives.size() << std::endl;
    for(int i = 0; i < (int)foundDrives.size(); i++)
        std::cout << "[" << i << "]\tID: " << foundDrives[i] << " (0x"<< std::hex << foundDrives[i] << std::dec << ")" << std::endl; 
}