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
    Candle::Candle(CANdleBaudrate_E canBaudrate)
    {
        shouldStopReceiver = false;
        usb = new UsbDevice();
        std::string setSerialCommand = "setserial " + usb->getSerialDeviceName() + " low_latency";
        if (system(setSerialCommand.c_str()) != 0)
        {
            std:: cout << "Could not execute command '" << setSerialCommand <<"'. Communication in low-speed mode." << std::endl;
            return;
        }
        if (!configCandleBaudrate(canBaudrate))
            std::cout << "Failed to set up CANdle baudrate @" << canBaudrate << "Mbps!" << std::endl;
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
    GenericMd80Frame _packMd80Frame(int canId, int msgLen, Md80FrameId_E canFrameId)
    {
        GenericMd80Frame frame;
        frame.frameId = USB_FRAME_MD80_GENERIC_FRAME;
        frame.driveCanId = canId;
        frame.canMsgLen = msgLen;
        frame.canMsg[0] = canFrameId;
        frame.canMsg[1] = 0x00;
        return frame;
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
#ifdef CANDLE_VERBOSE
                    std::cout << "Added Md80." << std::endl;
#endif        
                    for(int i = 0; i < md80s.size(); i++)
                        if(md80s[i].canId == canId)
                            return true;    //To avoid copies in the buffers
                    StdMd80CommandFrame_t newFrame = {.canId = canId, .toMd80 = {0, {0}}};
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
                std::cout << "No drives found." << std::endl;
#endif
                return false;   //No drives found at this baudrate
            }
#ifdef CANDLE_VERBOSE
            std::cout << "Found drives."  << std::endl;
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
        GenericMd80Frame frame = _packMd80Frame(canId, 10, Md80FrameId_E::FRAME_CAN_CONFIG);
        frame.frameId = USB_FRAME_MD80_CONFIG_CAN;
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
    bool Candle::configMd80Save(uint16_t canId)
    {
        GenericMd80Frame frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_CAN_SAVE);
        char tx[32];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 500))
            if (usb->rxBuffer[1] == true)
            {
#ifdef CANDLE_VERBOSE
                std::cout << "Saving in flash succesfull!" << std::endl;
#endif
                return true;
            }
#ifdef CANDLE_VERBOSE
        std::cout << "Saving in flash failed!" << std::endl;
#endif
        return false;
    }

    bool Candle::configMd80SetZero(uint16_t canId)
    {
        GenericMd80Frame frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_ZERO_ENCODER);
        char tx[32];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {
#ifdef CANDLE_VERBOSE
                std::cout << "Setting new zero position succesfull!" << std::endl;
#endif
                return true;
            }
#ifdef CANDLE_VERBOSE
        std::cout << "Setting new zero position failed!" << std::endl;
#endif
        return false;
    }
    bool Candle::configMd80SetCurrentLimit(uint16_t canId, float currentLimit)
    {
        GenericMd80Frame frame = _packMd80Frame(canId, 6, Md80FrameId_E::FRAME_BASE_CONFIG);
        *(float*)&frame.canMsg[2] = currentLimit;
        char tx[32];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[0] == USB_FRAME_MD80_GENERIC_FRAME && usb->rxBuffer[1] == true)
            {
#ifdef CANDLE_VERBOSE
                std::cout << "Setting new current limit succesfull!" << std::endl;
#endif
                return true;
            }
#ifdef CANDLE_VERBOSE
        std::cout << "Setting new current limit failed!" << std::endl;
#endif
        return false;
    }

    bool Candle::configCandleBaudrate(CANdleBaudrate_E canBaudrate)
    {
        char tx[10];
        tx[0] = USB_FARME_CANDLE_CONFIG_BAUDRATE;
        tx[1] = (uint8_t)canBaudrate;
        if(usb->transmit(tx, 2, true, 50))
            if(usb->rxBuffer[0] == USB_FARME_CANDLE_CONFIG_BAUDRATE && usb->rxBuffer[1] == true)
                return true;
        return false;
    }
    bool Candle::controlMd80Mode(uint16_t canId, Md80Mode_E mode)
    {
        GenericMd80Frame frame = _packMd80Frame(canId, 3, Md80FrameId_E::FRAME_CONTROL_SELECT);
        frame.canMsg[2] = mode;
        char tx[32];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {
#ifdef CANDLE_VERBOSE
                std::cout << "Seting control mode succesfull!" << std::endl;
#endif
                return true;
            }
#ifdef CANDLE_VERBOSE
        std::cout << "Seting control mode failed!" << std::endl;
#endif
        return false;
    }
    bool Candle::controlMd80Enable(uint16_t canId, bool enable)
    {
        GenericMd80Frame frame = _packMd80Frame(canId, 3, Md80FrameId_E::FRAME_MOTOR_ENABLE);
        frame.canMsg[2] = enable;
        char tx[32];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {
#ifdef CANDLE_VERBOSE
                std::cout << "Enabling succesfull!" << std::endl;
#endif
                return true;
            }
#ifdef CANDLE_VERBOSE
        std::cout << "Enabling failed!" << std::endl;
#endif
        return false;
    }
    void Candle::begin()
    {
        if(mode == CANdleMode_E::UPDATE)
            return; //TODO: Add printing?
        mode = CANdleMode_E::UPDATE;
        transmitterThread = std::thread(&Candle::transmit, this);
        receiverThread = std::thread(&Candle::receive, this);
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
        int length = 1 + md80s.size() * sizeof(StdMd80CommandFrame_t);
        memcpy(txBuffer, &stdFrame, length);
        usb->transmit(txBuffer, length, false);
    }
    bool Candle::isOk()
    {
        return true;
        return false;
    }


    
}

