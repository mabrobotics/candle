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
struct impedance_t
{
    float kp;
    float kd;
    float torque_ff;
};
struct pidx_t
{
    float kp, ki, kd, i_windup, output;
};
struct md80_t
{
    int adress;
    uint8_t controlMode;
    pidx_t velocityController;
    pidx_t positionController;
    impedance_t impedanceController;
};


struct CanFrame_t
{
    uint8_t length;
    uint8_t data[64];
};

struct StdMd80Frame_t
{
    uint16_t canId;
    uint8_t boolEnable;
    uint8_t mode;
    CanFrame_t toMd80;
};

struct stdUsbFrame_t
{
    uint8_t id;
    std::vector<md80_t> md80s;
};

#pragma pack(pop)

namespace mab
{
    Candle::Candle(std::string canalizatorDev)
    {
        shouldStopReceiver = false;
        usb = new UsbDevice(canalizatorDev.c_str());
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
        ConfigFrame_t cfg = {CONFIG_CANDLE, canBaud, canUpdate, usbUpdate};
        if(usb->transmit((char*)&cfg, sizeof(cfg), true))
            if(usb->rxBuffer[0] == CONFIG_CANDLE)
                return true;
        return false;
    }
    bool Candle::addMd80(int adress)
    {
        AddMd80Frame_t add = {ADD_MD80, adress};
        usb->transmit((char*)&add, sizeof(AddMd80Frame_t));
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
    void Candle::transmitNewStdFrame()
    {

    }
    bool Candle::isOk()
    {
        return true;
        return false;
    }


    
}

