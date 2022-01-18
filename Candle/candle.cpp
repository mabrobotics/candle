#include "candle.hpp"

#include <cstring>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include "unistd.h"

#define CAN_DEFAULT_SPEED 1000000

enum UsbFrameId_t : uint8_t
{
    NONE,
    CONFIG_CANDLE,
    ADD_MD80,
    CONFIG_MD80,
};

#pragma pack(push, 1)   //Ensures there in no padding (dummy) bytes in the structures below
struct ConfigFrame_t
{
    uint8_t id;
    uint32_t CanBaudrate;
    uint32_t CanUpdateRate;
    uint32_t UsbUpdateRate;
};
struct AddMd80Frame_t
{
    uint8_t id;
    uint16_t driveAdress;
};
struct ConfigMd80Frame_t
{
    uint8_t id;
    uint16_t driveAdress;
    mab::MD80_mode control_mode;
    float maxCurrent;
};
#pragma pack(pop)

namespace mab
{
    Candle::Candle()
    {
        shouldStopReceiver = false;
        usb = new UsbDevice();
        if (usb == nullptr)
        {
            std:: cout << "Failed to open port " << canalizatorDev << std::endl;
            return;
        }
        std::string setSerialCommand = "setserial " + canalizatorDev + " low_latency";
        if (system(setSerialCommand.c_str()) != 0)
        {
            std:: cout << "Could not execute command '" << setSerialCommand <<"'. Communication in low-speed mode." << std::endl;
            return;
        }
        receiverThread = std::thread(&Candle::receive, this);
    }
    Candle::~Candle()
    {
        shouldStopReceiver = true;
        receiverThread.join();
    }
    void Candle::receive()
    {
        while(!shouldStopReceiver)
        {
            if(usb->receive())
            {
                std::cout << "Got " << std::dec << usb->bytesReceived << "bytes." <<std::endl;
                std::cout << usb->rxBuffer << std::endl;
                for(int i = 0; i < usb->bytesReceived; i++)
                    std::cout << std::hex << "0x" << (int)usb->rxBuffer[i] << " ";
                std::cout << std::endl << "#######################################################" << std::endl; 
            }
        }
    }
    bool Candle::transmitConfig(int canBaud, int canUpdate, int usbUpdate)
    {
		if(inUpdateMode())
			return false;
        ConfigFrame_t cfg = {CONFIG_CANDLE, canBaud, canUpdate, usbUpdate};
        if(usb->transmit((char*)&cfg, sizeof(cfg), true))
            if(usb->rxBuffer[0] == CONFIG_CANDLE)
                return true;
        return false;
    }
    bool Candle::addMd80(int adress)
    {
		if(inUpdateMode())
			return false;
        AddMd80Frame_t add = {ADD_MD80, adress};
        if(usb->transmit((char*)&add, sizeof(AddMd80Frame_t), true))
            if(usb->rxBuffer[0] == ADD_MD80)
                if(usb->rxBuffer[1] == true)
                {
                    StdMd80Frame_t newFrame = {.canId = adress, .toMd80 = {0}};
                    stdFrame.md80Frames.push_back(newFrame);
                    md80_t newMd80 = {.adress = adress, .controlMode = 0,
						.velocityController = {0},
                    	.positionController = {0}, 
                    	.impedanceController = {0}};	//TODO: Should be filled with defaults? 
					md80s.push_back(newMd80);
                }
        return false;
    }
    bool Candle::configMd80(int adress, float max_current, mab::MD80_mode mode)
    {
        ConfigMd80Frame_t cfg = {0x03, adress, mode, max_current};
        usb->transmit((char*)&cfg, sizeof(ConfigMd80Frame_t));
    }
    void Candle::begin()
    {
        mode = CANdle_mode::UPDATE;
    }
    void Candle::end()
    {
        mode = CANdle_mode::CONFIG;
    }
	bool Candle::inUpdateMode()
	{
		if(mode == CANdle_mode::UPDATE)
		{
			std::cout << "Invalid Action. CANDLE is currently in UPDATE mode, and requested action requires CONFIG mode!" << std::endl;
			std::cout << "Change mode by using `Candle.end()`" << std::endl;
			return true;
		}
		return false;
	}
	bool Candle::inConfigMode()
	{
		if(mode == CANdle_mode::CONFIG)
		{
			std::cout << "Invalid Action. CANDLE is currently in CONFIG mode, and requested action requires UPDATE mode!" << std::endl;
			std::cout << "Change mode by using `Candle.begin()`" << std::endl;
			return true;
		}
		return false;
	}
    void Candle::transmitNewStdFrame()
    {

    }
    bool Candle::isOk()
    {
        return true;
        return false;
    }


    
}

