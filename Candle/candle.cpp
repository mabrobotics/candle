#include "candle.hpp"

#include <cstring>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include "candle_protocol.hpp"

#define CANDLE_VERBOSE
namespace mab
{
    Candle::Candle()
    {
        shouldStopReceiver = false;
        usb = new UsbDevice();
        std::string setSerialCommand = "setserial " + usb->getSerialDeviceName() + " low_latency";
        if (system(setSerialCommand.c_str()) != 0)
        {
            std:: cout << "Could not execute command '" << setSerialCommand <<"'. Communication in low-speed mode." << std::endl;
            return;
        }
        //receiverThread = std::thread(&Candle::receive, this);
    }
    Candle::~Candle()
    {
        shouldStopReceiver = true;
        shouldStopTransmitter = true;
        if(receiverThread.joinable())
            receiverThread.join();
        if(transmitterThread.joinable())
            transmitterThread.join();
    }
    void Candle::receive()
    {
        while(!shouldStopReceiver)
        {
            if(usb->receive())
            {
                //parse
            }
        }
    }
    void Candle::transmit()
    {
        while(!shouldStopTransmitter)
        {
            transmitNewStdFrame();
            usleep(1000);
        }
    }
    bool Candle::transmitConfig(int canBaud, int canUpdate, int usbUpdate)
    {
		// if(inUpdateMode())
		// 	return false;
        // ConfigFrame_t cfg = {USB_FRAME, (uint32_t)canBaud, (uint32_t)canUpdate, (uint32_t)usbUpdate};
        // if(usb->transmit((char*)&cfg, sizeof(cfg), true))
        //     if(usb->rxBuffer[0] == CONFIG_CANDLE)
        //         return true;
        return false;
    }
    bool Candle::addMd80(uint16_t canId)
    {
		if(inUpdateMode())
			return false;
        AddMd80Frame_t add = {USB_FRAME_MD80_ADD, canId};
        if(usb->transmit((char*)&add, sizeof(AddMd80Frame_t), true))
            if(usb->rxBuffer[0] == USB_FRAME_MD80_ADD)
                if(usb->rxBuffer[1] == true)
                {
                    StdMd80Frame_t newFrame = {.canId = canId, .toMd80 = {0, {0}}};
                    stdFrame.md80Frames.push_back(newFrame);
                    md80_t newMd80 = {.canId = canId, .controlMode = 0,
						.velocityController = {0,0,0,0},
                    	.positionController = {0,0,0,0}, 
                    	.impedanceController = {0,0,0}};	//TODO: Should be filled with defaults? 
					md80s.push_back(newMd80);
                    return true;
                }
        return false;
    }
    bool Candle::configMd80(uint16_t canId, float max_current, mab::Md80Mode_E mode)
    {
        ConfigMd80Frame_t cfg = {0x03, canId, mode, max_current};
        char tx[128];
        usb->transmit(tx, 127, true);
        return true; //TODO: Acknowleagement
    }
    bool Candle::ping()
    {
        char tx[128];
        tx[0] = USB_FRAME_PING_START;
        tx[1] = 0x00;
        if(usb->transmit(tx, 2, true, 2500))    //Scanning 2047 FDCAN ids, takes ~2100ms, thus wait for 2.5 sec
        {
            uint16_t*ids = (uint16_t*)&usb->rxBuffer[1];
            if(ids[0] == 0)
            {
#ifdef CANDLE_VERBOSE
                std::cout << "No drives found @" << std::to_string(baudrateMbps) << "Mbps" << std::endl;
#endif
                return false;   //No drives found at this baudrate
            }
#ifdef CANDLE_VERBOSE
            std::cout << "Found drives @" << std::to_string(baudrateMbps) << "Mbps" << std::endl;
            for(int i = 0; i < 16; i++)
            {
                if (ids[i] == 0)
                    break;  //No more ids in the message
                std::cout << std::to_string(i+1) <<": ID = " << ids[i]  << 
                    " (0x" << std::hex << ids[i] << std::dec << ")" << std::endl;
            }
#endif
            return true;
        }
        return false;
    }

    bool Candle::configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout)
    {
        GenericMd80Frame frame;
        frame.frameId = USB_FRAME_MD80_CONFIG_CAN;
        frame.driveCanId = canId;
        frame.canMsgLen = 10;
        frame.canMsg[0] = Md80FrameId_E::FRAME_CAN_CONFIG;
        frame.canMsg[1] = 0x00;
        *(uint16_t*)&frame.canMsg[2] = newId;
        *(uint32_t*)&frame.canMsg[4] = newBaudrateMbps * 1000000;
        *(uint16_t*)&frame.canMsg[8] = newTimeout;
        char tx[63];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if (usb->transmit(tx, len, true, 100))
            if(usb->rxBuffer[1] == 1)
            {
#ifdef CANDLE_VERBOSE
                std::cout << "CAN config change succesfull!" << std::endl;
                std::cout << "Drive ID = " << std::to_string(canId) << " was changed to ID = " << std::to_string(newId) << std::endl;
                std::cout << "It's baudrate is now " << std::to_string(newBaudrateMbps) << "Mbps" << std::endl;
                std::cout << "It's CAN timeout (watchdog) is now " << (newTimeout == 0 ? "Disabled" : std::to_string(newTimeout) + "ms")  << std::endl;
#endif
                return true;
            }
#ifdef CANDLE_VERBOSE
                std::cout << "CAN config change failed!" << std::endl;
#endif
        return false;
    }
    void Candle::begin()
    {
        if(mode == CANdleMode_E::UPDATE)
            return; //TODO: Add printing?
        mode = CANdleMode_E::UPDATE;
        transmitterThread = std::thread(&Candle::transmit, this);
    }
    void Candle::end()
    {
        if(mode == CANdleMode_E::CONFIG)
            return;
        mode = CANdleMode_E::CONFIG;
        shouldStopTransmitter = true;
        transmitterThread.join();
    }
	bool Candle::inUpdateMode()
	{
		if(mode == CANdleMode_E::UPDATE)
		{
			std::cout << "Invalid Action. CANDLE is currently in UPDATE mode, and requested action requires CONFIG mode!" << std::endl;
			std::cout << "Change mode by using `Candle.end()`" << std::endl;
			return true;
		}
		return false;
	}
	bool Candle::inConfigMode()
	{
		if(mode == CANdleMode_E::CONFIG)
		{
			std::cout << "Invalid Action. CANDLE is currently in CONFIG mode, and requested action requires UPDATE mode!" << std::endl;
			std::cout << "Change mode by using `Candle.begin()`" << std::endl;
			return true;
		}
		return false;
	}
    void Candle::transmitNewStdFrame()
    {
        char txBuffer[512];
        stdFrame.id = 0x05;
        for(int i = 0; i < (int)md80s.size(); i++)
        {
            switch (md80s[i].controlMode)
            {
            case Md80Mode_E::IDLE:
                stdFrame.md80Frames[i].canId = md80s[i].canId;
                stdFrame.md80Frames[i].toMd80.length = 2;
                stdFrame.md80Frames[i].toMd80.data[0] = Md80FrameId_E::FRAME_GET_INFO;
                break;
            case Md80Mode_E::IMPEDANCE:
                stdFrame.md80Frames[i].canId = md80s[i].canId;
                stdFrame.md80Frames[i].toMd80.length = 32;
                stdFrame.md80Frames[i].toMd80.data[0] = Md80FrameId_E::FRAME_IMP_CONTROL;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[2] = md80s[i].impedanceController.kp;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[6] = md80s[i].impedanceController.kd;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[10] = md80s[i].positionTarget;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[14] = md80s[i].velocityTarget;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[18] = md80s[i].torqueSet;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[22] = md80s[i].maxTorque;
                break;
            case Md80Mode_E::POSITION_PID:
                stdFrame.md80Frames[i].canId = md80s[i].canId;
                stdFrame.md80Frames[i].toMd80.length = 32;
                stdFrame.md80Frames[i].toMd80.data[0] = Md80FrameId_E::FRAME_POS_CONTROL;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[2] = md80s[i].positionController.kp;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[6] = md80s[i].positionController.ki;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[10] = md80s[i].positionController.kd;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[14] = md80s[i].positionController.i_windup;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[18] = md80s[i].maxTorque;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[22] = md80s[i].positionTarget;
                break;
            case Md80Mode_E::VELOCITY_PID:
                stdFrame.md80Frames[i].canId = md80s[i].canId;
                stdFrame.md80Frames[i].toMd80.length = 32;
                stdFrame.md80Frames[i].toMd80.data[0] = Md80FrameId_E::FRAME_VEL_CONTROL;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[2] = md80s[i].velocityController.kp;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[6] = md80s[i].velocityController.ki;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[10] = md80s[i].velocityController.kd;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[14] = md80s[i].velocityController.i_windup;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[18] = md80s[i].maxVelocity;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[22] = md80s[i].velocityTarget;
                break;
            case Md80Mode_E::TORQUE:
                stdFrame.md80Frames[i].canId = md80s[i].canId;
                stdFrame.md80Frames[i].toMd80.length = 32;
                stdFrame.md80Frames[i].toMd80.data[0] = Md80FrameId_E::FRAME_IMP_CONTROL;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[2] = 0.0f;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[6] = 0.0f;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[10] = 0.0f;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[14] = 0.0f;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[18] = md80s[i].torqueSet;
                *(float*)&stdFrame.md80Frames[i].toMd80.data[22] = md80s[i].maxTorque;
                break;
            default:
                break;
            }
        }
        int length = 1 + md80s.size() * sizeof(StdMd80Frame_t);
        memcpy(txBuffer, &stdFrame, length);
        usb->transmit(txBuffer, length, false);
    }
    bool Candle::isOk()
    {
        return true;
        return false;
    }


    
}

