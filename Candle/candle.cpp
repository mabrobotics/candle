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
                char*rx = usb->rxBuffer;
                if(rx[0] == USB_FRAME_UPDATE)
                    for(int i = 0; i < md80s.size(); i++)
                        md80s[i].updateResponseData((StdMd80ResponseFrame_t*)rx[1 + i * sizeof(StdMd80ResponseFrame_t)]);
            }
        }
    }
    void Candle::transmit()
    {
        while(!shouldStopTransmitter)
        {
            transmitNewStdFrame();
            usleep(10000);
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
					md80s.push_back(Md80(canId));
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
    Md80* Candle::getMd80FromList(uint16_t id)
    {
        for(int i = 0; i < md80s.size(); i++)
            if(md80s[i].getId() == id)
                return &md80s[i];
        return nullptr;
    }
    bool Candle::controlMd80Mode(uint16_t canId, Md80Mode_E mode)
    {
        Md80*drive = getMd80FromList(canId);
        if(drive == nullptr)
        {
#ifdef CANDLE_VERBOSE
            std::cout << "Drive not found in the Candle database. Use Candle.addMd80() to add drives first!" << std::endl;
#endif
            return false;
        }
        GenericMd80Frame frame = _packMd80Frame(canId, 3, Md80FrameId_E::FRAME_CONTROL_SELECT);
        frame.canMsg[2] = mode;
        char tx[32];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {
#ifdef CANDLE_VERBOSE
                std::cout << "Setting control mode succesfull!" << std::endl;
#endif
                drive->setControlMode(mode);
                return true;
            }
#ifdef CANDLE_VERBOSE
        std::cout << "Setting control mode failed!" << std::endl;
#endif
        return false;
    }
    bool Candle::controlMd80Enable(uint16_t canId, bool enable)
    {
        Md80*drive = getMd80FromList(canId);
        if(drive == nullptr)
        {
#ifdef CANDLE_VERBOSE
            std::cout << "Drive not found in the Candle database. Use Candle.addMd80() to add drives first!" << std::endl;
#endif
            return false;
        }
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
    bool Candle::begin()
    {
        if(mode == CANdleMode_E::UPDATE)
            return false; //TODO: Add printing?
        char tx[128];
        tx[0] = USB_FRAME_BEGIN;
        tx[1] = 0x00;
        if(usb->transmit(tx, 2, true, 10))
        {
            mode = CANdleMode_E::UPDATE;
            transmitterThread = std::thread(&Candle::transmit, this);
            receiverThread = std::thread(&Candle::receive, this);
            return true;
        }
        return false;

    }
    bool Candle::end()
    {
        if(mode == CANdleMode_E::CONFIG)
            return false;
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
        char tx[512];
        tx[0] = USB_FRAME_UPDATE;
        for(int i = 0; i < (int)md80s.size(); i++)
        {
            md80s[i].updateCommandFrame();
            *(StdMd80CommandFrame_t*)&tx[1 + i*sizeof(StdMd80CommandFrame_t)] = md80s[i].getCommandFrame();
        }
        
        int length = 1 + md80s.size() * sizeof(StdMd80CommandFrame_t);
        usb->transmit(tx, length, false);
    }
    bool Candle::isOk()
    {
        return true;
        return false;
    }    
}
