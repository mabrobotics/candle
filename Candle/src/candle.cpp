#include "candle.hpp"

#include <cstring>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include "candle_protocol.hpp"

namespace mab
{
    class mystreambuf: public std::streambuf {    };
    mystreambuf nostreambuf;
    std::ostream nocout(&nostreambuf);
    #define vout ((this->printVerbose)? std::cout << "[CANDLE] " : nocout)

    uint64_t getTimestamp() 
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    }

    Candle::Candle(CANdleBaudrate_E canBaudrate, bool _printVerbose)
    {
        vout << "CANdle library version: " << getVersion() << std::endl;
        vout << "Creating CANdle object." << std::endl;
        printVerbose = _printVerbose;
        usb = new UsbDevice();
        std::string setSerialCommand = "setserial " + usb->getSerialDeviceName() + " low_latency";
        if (system(setSerialCommand.c_str()) != 0)
            std:: cout << "Could not execute command '" << setSerialCommand <<"'. Communication in low-speed mode." << std::endl;
        this->reset();
        usleep(100000);
        if (!configCandleBaudrate(canBaudrate))
            vout << "Failed to set up CANdle baudrate @" << canBaudrate << "Mbps!" << std::endl;
        vout << "CANdle ready." << std::endl;
    }
    Candle::~Candle()
    {
        if(this->inUpdateMode())
            this->end();
    }
    const std::string Candle::getVersion()
    {
        return version;
    }
    void Candle::receive()
    {
        while(!shouldStopReceiver)
        {
            if(usb->receive())
            {
                if(usb->rxBuffer[0] == USB_FRAME_UPDATE)
                {
                    for(int i = 0; i < (int)md80s.size(); i++)
                        md80s[i].__updateResponseData((StdMd80ResponseFrame_t*)&usb->rxBuffer[1 + i * sizeof(StdMd80ResponseFrame_t)]);
                }
            }
        }
    }
    void Candle::transmit()
    {
        while(!shouldStopTransmitter)
        {
            transmitNewStdFrame();
            msgsSent++;
            usleep(10000);
        }
    }
    GenericMd80Frame32 _packMd80Frame(int canId, int msgLen, Md80FrameId_E canFrameId)
    {
        GenericMd80Frame32 frame;
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
                    vout << "Added Md80." << std::endl;
					md80s.push_back(Md80(canId));
                    return true;
                }
        return false;
    }
    std::vector<uint16_t> Candle::ping()
    {
        vout << "Starting pinging drives..." << std::endl;
        char tx[128];
        tx[0] = USB_FRAME_PING_START;
        tx[1] = 0x00;
        std::vector<uint16_t> ids;
        if(usb->transmit(tx, 2, true, 2500))    //Scanning 2047 FDCAN ids, takes ~2100ms, thus wait for 2.5 sec
        {
            uint16_t*idsPointer = (uint16_t*)&usb->rxBuffer[1];
            for(int i = 0; i < MAX_DEVICES; i++)
            {
                uint16_t id = idsPointer[i];
                if(id == 0x00)
                    break;
                ids.push_back(id);
            }
            if(ids.size() == 0)
            {
                vout << "No drives found." << std::endl;
                return ids;
            }
            vout << "Found drives."  << std::endl;
            for(size_t i = 0; i < ids.size(); i++)
            {
                if (ids[i] == 0)
                    break;  //No more ids in the message
                
                vout << std::to_string(i+1) <<": ID = " << ids[i]  << 
                    " (0x" << std::hex << ids[i] << std::dec << ")" << std::endl;
                if(ids[i] > 2047)
                {
                    vout << "Error! This ID is invalid! Probably two or more drives share same ID." <<
                        "Communication will most likely be broken until IDs are unique!" << std::endl;
                    std::vector<uint16_t>empty;
                    return empty;
                }
            }
        }
        return ids;
    }
    bool Candle::sengGenericFDCanFrame(uint16_t canId, int msgLen, const char*txBuffer, char*rxBuffer, int timeoutMs)
    {
        int fdcanTimeout = timeoutMs - 3;
        if(timeoutMs < 3)
        {
            timeoutMs = 3;
            fdcanTimeout = 1;
        }
        GenericMd80Frame64 frame;
        frame.frameId = mab::UsbFrameId_t::USB_FRAME_MD80_GENERIC_FRAME;
        frame.driveCanId = canId;
        frame.canMsgLen = msgLen;
        frame.timeoutMs = fdcanTimeout;
        memcpy(frame.canMsg, txBuffer, msgLen);
        char tx[96];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if (usb->transmit(tx, len, true, timeoutMs))    //Got some response
        {
            if(usb->rxBuffer[0] == tx[0] && // USB Frame ID matches
                usb->rxBuffer[1] == true &&
                usb->bytesReceived <= 64 + 2)   // response can ID matches
            {
                memcpy(rxBuffer, &usb->rxBuffer[2], usb->bytesReceived - 2);
                return true;
            }
        }
        return false;
    }


    bool Candle::configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 10, Md80FrameId_E::FRAME_CAN_CONFIG);
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
                vout << "CAN config change successfull!" << std::endl;
                vout << "Drive ID = " << std::to_string(canId) << " was changed to ID = " << std::to_string(newId) << std::endl;
                vout << "It's baudrate is now " << std::to_string(newBaudrateMbps) << "Mbps" << std::endl;
                vout << "It's CAN timeout (watchdog) is now " << (newTimeout == 0 ? "Disabled" : std::to_string(newTimeout) + "ms")  << std::endl;
                return true;
            }
                vout << "CAN config change failed!" << std::endl;
        return false;
    }
    bool Candle::configMd80Save(uint16_t canId)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_CAN_SAVE);
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 500))
            if (usb->rxBuffer[1] == true)
            {
                vout << "Saving in flash successfull at ID = " << canId << std::endl;
                return true;
            }
        vout << "Saving in flash failed at ID = " << canId << std::endl;
        return false;
    }

    bool Candle::controlMd80SetEncoderZero(uint16_t canId)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_ZERO_ENCODER);
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {
                vout << "Setting new zero position successfull at ID = " << canId << std::endl;
                return true;
            }
        vout << "Setting new zero position failed at ID = " << canId << std::endl;
        return false;
    }
    bool Candle::configMd80SetCurrentLimit(uint16_t canId, float currentLimit)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 6, Md80FrameId_E::FRAME_BASE_CONFIG);
        *(float*)&frame.canMsg[2] = currentLimit;
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[0] == USB_FRAME_MD80_GENERIC_FRAME && usb->rxBuffer[1] == true)
            {
                vout << "Setting new current limit successfull at ID = " << canId << std::endl;
                return true;
            }
        vout << "Setting new current limit failed at ID = " << canId << std::endl;
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
    Md80& Candle::getMd80FromList(uint16_t id)
    {
        for(int i = 0; i < (int)md80s.size(); i++)
            if(md80s[i].getId() == id)
                return md80s[i];
        throw "getMd80FromList(id): Id not found on the list!";
    }
    bool Candle::controlMd80SetEncoderZero(Md80&drive)
    {
        return this->controlMd80SetEncoderZero(drive.getId());
    }
    bool Candle::controlMd80Enable(Md80&drive, bool enable)
    {
        return this->controlMd80Enable(drive.getId(), enable);
    }
    bool Candle::controlMd80Mode(Md80&drive, Md80Mode_E mode)
    {
        return this->controlMd80Mode(drive.getId(), mode);
    }
    bool Candle::controlMd80Mode(uint16_t canId, Md80Mode_E mode)
    {
        try
        {
            Md80&drive = getMd80FromList(canId);
            GenericMd80Frame32 frame = _packMd80Frame(canId, 3, Md80FrameId_E::FRAME_CONTROL_SELECT);
            frame.canMsg[2] = mode;
            char tx[64];
            int len = sizeof(frame);
            memcpy(tx, &frame, len);
            if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {
                vout << "Setting control mode successfull at ID = " << canId << std::endl;
                drive.__setControlMode(mode);
                return true;
            }
            vout << "Setting control mode failed at ID = " << canId << std::endl;
            return false;
        }
        catch (const char*msg)
        {
            vout << msg << std::endl;
            return false;
        }        
    }
    bool Candle::controlMd80Enable(uint16_t canId, bool enable)
    {
        try
        {
            GenericMd80Frame32 frame = _packMd80Frame(canId, 3, Md80FrameId_E::FRAME_MOTOR_ENABLE);
            frame.canMsg[2] = enable;
            char tx[64];
            int len = sizeof(frame);
            memcpy(tx, &frame, len);
            if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {   
                if(enable)
                    vout << "Enabling successfull at ID = " << canId << std::endl;
                else
                {
                    vout << "Disabling successfull at ID = " << canId << std::endl;
                    this->getMd80FromList(canId).__updateRegulatorsAdjusted(false);  //Drive will operate at default params
                }
                return true;
            }
            vout << "Enabling/Disabling failed at ID = " << canId << std::endl;
            return false;
        }
        catch(const char*msg)
        {
            vout << msg << std::endl;
            return false;
        }        
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
            vout << "Begginig auto update loop mode" << std::endl;
            mode = CANdleMode_E::UPDATE;
            shouldStopTransmitter = false;
            shouldStopReceiver = false;
            msgsSent = 0;
            msgsReceived = 0;
            transmitterThread = std::thread(&Candle::transmit, this);
            receiverThread = std::thread(&Candle::receive, this);
            return true;
        }
        vout << "Failed to begin auto update loop mode" << std::endl;
        return false;
    }
    bool Candle::end()
    {
        if(mode == CANdleMode_E::CONFIG)
            return false;
        shouldStopTransmitter = true;
        shouldStopReceiver = true;
        transmitterThread.join();
        receiverThread.join();
        
        char tx[128];
        tx[0] = USB_FRAME_END;
        tx[1] = 0x00;
        if(usb->transmit(tx, 2, true, 10))
            mode = CANdleMode_E::CONFIG;
        vout << "Ending auto update loop mode" << std::endl;

        return true;
    }
    bool Candle::reset()
    {
        char tx[128];
        tx[0] = USB_FRAME_RESET;
        tx[1] = 0x00;
        if(usb->transmit(tx, 2, true, 100))
        {
            vout << "Reset successfull!" << std::endl;
            return true;
        }
        vout << "Reset failed!" << std::endl;
        return false;
    }
	bool Candle::inUpdateMode()
	{
		if(mode == CANdleMode_E::UPDATE)
			return true;
		return false;
	}
	bool Candle::inConfigMode()
	{
		if(mode == CANdleMode_E::CONFIG)
			return true;
		return false;
	}
    void Candle::transmitNewStdFrame()
    {
        char tx[512];
        tx[0] = USB_FRAME_UPDATE;
        for(int i = 0; i < (int)md80s.size(); i++)
        {
            md80s[i].__updateCommandFrame();
            *(StdMd80CommandFrame_t*)&tx[1 + i*sizeof(StdMd80CommandFrame_t)] = md80s[i].__getCommandFrame();
        }
        
        int length = 1 + md80s.size() * sizeof(StdMd80CommandFrame_t);
        usb->transmit(tx, length, false);
    }

    bool Candle::setupMd80Calibration(uint16_t canId)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_CALIBRATION);
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {
                vout << "Starting calibration at ID = " << canId << std::endl;
                return true;
            }
        vout << "Starting calibration failed at ID = " << canId << std::endl;
        return false;
    }
    bool Candle::setupMd80Diagnostic(uint16_t canId)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_DIAGNOSTIC);
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if(usb->transmit(tx, len, true, 50))
        {
            std::cout << "[CANDLE] Library version: " << getVersion() << std::endl;
            std::cout << "[CANDLE] DIAG at ID = " << canId << ": " << std::string(&usb->rxBuffer[2]) << std::endl;
            return true;
        }
        vout << "Diagnostic failed at ID = " << canId << std::endl;
        return false;
    }
}
