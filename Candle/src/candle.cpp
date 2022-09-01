#include "candle.hpp"
#include "candle_protocol.hpp"

#include <cstring>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <vector>

namespace mab
{
    class mystreambuf : public std::streambuf
    {
    };

    mystreambuf nostreambuf;
    std::ostream nocout(&nostreambuf);
#define vout ((this->printVerbose) ? std::cout << "[CANDLE] " : nocout)
    std::string statusOK = "  [OK]";
    std::string statusFAIL = "  [FAILED]";

    uint64_t getTimestamp()
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    }

    std::vector<Candle *> Candle::instances = std::vector<Candle *>();

    Candle::Candle(CANdleBaudrate_E canBaudrate, bool _printVerbose, bool useLogs, mab::CANdleFastMode_E _fastMode, bool printFailure)
    {
        printVerbose = _printVerbose;
        auto listOfCANdle = UsbDevice::getConnectedACMDevices("MAB_Robotics", "MD_USB-TO-CAN");
        if (listOfCANdle.size() == 0)
            vout << "No CANdle found!" << std::endl;
        if (instances.size() == 0)
            usb = new UsbDevice(listOfCANdle[0], "MAB_Robotics", "MD_USB-TO-CAN");
        else
        {
            for (auto &entry : listOfCANdle)
            {
                unsigned int newIdCount = 0;
                for (auto instance : instances)
                {
                    if (UsbDevice::getConnectedDeviceId(entry) != instance->getUsbDeviceId())
                        newIdCount++;
                }
                /* only if all instances were different from the current one -> create new device */
                if (newIdCount == instances.size())
                {
                    usb = new UsbDevice(entry, "MAB_Robotics", "MD_USB-TO-CAN");
                    candleId = newIdCount;
                    goto loopdone; // Only legit use of goto left in C++
                }
            }
            if (printFailure)
                vout << "Failed to create CANdle object." << statusFAIL << std::endl;
            throw "Failed to create CANdle object";
            return;
        }
    loopdone:
        vout << "CANdle library version: " << getVersion() << std::endl;
        std::string setSerialCommand = "setserial " + usb->getSerialDeviceName() + " low_latency";
        if (system(setSerialCommand.c_str()) != 0)
            std::cout << "Could not execute command '" << setSerialCommand << "'. Communication in low-speed mode." << std::endl;
        this->reset();
        usleep(100000);
        if (!configCandleBaudrate(canBaudrate, true))
            vout << "Failed to set up CANdle baudrate @" << canBaudrate << "Mbps!" << std::endl;
        vout << "CANdle at " << this->usb->getSerialDeviceName() << ", ID: 0x" << std::hex << this->getUsbDeviceId() << std::dec << " ready." << std::endl;
        _useLogs = useLogs;
        fastMode = _fastMode;
        switch (fastMode)
        {
        case CANdleFastMode_E::FAST1:
            maxDevices = (int)CANdleMaxDevices_E::MAX_DEV_FAST1;
            break;
        case CANdleFastMode_E::FAST2:
            maxDevices = (int)CANdleMaxDevices_E::MAX_DEV_FAST2;
            break;
        default:
            maxDevices = (int)CANdleMaxDevices_E::MAX_DEV_NORMAL;
            break;
        }
        Candle::instances.push_back(this);
    }

    Candle::~Candle()
    {
        if (this->inUpdateMode())
            this->end();
    }

    void Candle::updateModeBasedOnMd80List()
    {
        if (md80s.size() <= (int)CANdleMaxDevices_E::MAX_DEV_FAST2)
        {
            fastMode = CANdleFastMode_E::FAST2;
            vout << "Set current speed mode to FAST2" << std::endl;
        }
        else if (md80s.size() <= (int)CANdleMaxDevices_E::MAX_DEV_FAST1)
        {
            fastMode = CANdleFastMode_E::FAST1;
            vout << "Set current speed mode to FAST1" << std::endl;
        }
        else if (md80s.size() <= (int)CANdleMaxDevices_E::MAX_DEV_NORMAL)
        {
            fastMode = CANdleFastMode_E::NORMAL;
            vout << "Set current speed mode to NORMAL" << std::endl;
        }
    }

    const std::string Candle::getVersion()
    {
        return version;
    }

    int Candle::getActualCommunicationFrequency()
    {
        return (int)this->usbCommsFreq;
    }

    void Candle::receive()
    {
        while (!shouldStopReceiver)
        {
            if (usb->receive())
            {
                if (usb->rxBuffer[0] == USB_FRAME_UPDATE)
                {
                    uint64_t nsec = std::chrono::duration_cast<nsec_t>(std::chrono::system_clock::now().time_since_epoch()).count();
                    double timeInSec = nsec * 1e-9;
                    if (_useLogs)
                        receiveLogFile << std::to_string(receive_count) << "," << std::to_string(timeInSec);
                    for (int i = 0; i < (int)md80s.size(); i++)
                    {
                        StdMd80ResponseFrame_t *frame = (StdMd80ResponseFrame_t *)&usb->rxBuffer[1 + i * sizeof(StdMd80ResponseFrame_t)];
                        md80s.at(frame->canId).__updateResponseData(frame, timeInSec, receive_count);
                        if (_useLogs)
                        {
                            auto motorStatus = md80s.at(frame->canId).getMotorStatus();
                            receiveLogFile << "," << std::to_string(frame->canId) << ":" << std::to_string(motorStatus["position"])
                                           << " " << std::to_string(motorStatus["velocity"])
                                           << " " << std::to_string(motorStatus["torque"])
                                           << " " << std::to_string(motorStatus["temperature"]);
                        }
                    }
                    if (_useLogs)
                        receiveLogFile << std::endl;

                    receive_count++;
                }
            }
        }
    }

    void Candle::transmit()
    {
        int txCounter = 0;
        uint64_t freqCheckStart = getTimestamp();
        while (!shouldStopTransmitter)
        {
            if (++txCounter == 250)
            {
                this->usbCommsFreq = 250.0 / (float)(getTimestamp() - freqCheckStart) * 1000.0f;
                freqCheckStart = getTimestamp();
                txCounter = 0;
            }
            transmitNewStdFrame();
            msgsSent++;
            switch (fastMode)
            {
            case CANdleFastMode_E::FAST1:
                usleep(1990 * 2);
                break;
            case CANdleFastMode_E::FAST2:
                usleep(1950);
                break;
            default:
                usleep(10000);
                break;
            }
        }
    }

    void Candle::setVebose(bool enable)
    {
        printVerbose = enable;
    }

    unsigned long Candle::getUsbDeviceId()
    {
        return usb->getId();
    }

    GenericMd80Frame32 _packMd80Frame(int canId, int msgLen, Md80FrameId_E canFrameId)
    {
        GenericMd80Frame32 frame;
        frame.frameId = USB_FRAME_MD80_GENERIC_FRAME;
        frame.driveCanId = canId;
        frame.canMsgLen = msgLen;
        frame.timeoutMs = 10;
        frame.canMsg[0] = canFrameId;
        frame.canMsg[1] = 0x00;
        return frame;
    }

    void Candle::sendGetInfoFrame(mab::Md80 &drive)
    {
        GenericMd80Frame32 getInfoFrame = _packMd80Frame(drive.getId(), 2, Md80FrameId_E::FRAME_GET_INFO);
        if (usb->transmit((char *)&getInfoFrame, sizeof(getInfoFrame), true, 100))
        {
            uint8_t cheaterBuffer[72];
            memcpy(&cheaterBuffer[1], usb->rxBuffer, usb->bytesReceived);
            *(uint16_t *)&cheaterBuffer[0] = drive.getId();
            cheaterBuffer[2] = 16; // Cheater buffer is a dirty trick to make USB_FRAME_MD80_GENERIC_FRAME response compatibile with __updateResponseData
            drive.__updateResponseData((mab::StdMd80ResponseFrame_t *)cheaterBuffer);
        }
    }

    void Candle::sendMotionCommand(mab::Md80 &drive, float pos, float vel, float torque)
    {
        GenericMd80Frame32 motionCommandFrame = _packMd80Frame(drive.getId(), 16, Md80FrameId_E::FRAME_SET_MOTION_TARGETS);
        *(float *)&motionCommandFrame.canMsg[2] = vel;
        *(float *)&motionCommandFrame.canMsg[6] = pos;
        *(float *)&motionCommandFrame.canMsg[10] = torque;
        if (usb->transmit((char *)&motionCommandFrame, sizeof(motionCommandFrame), true, 100))
        {
            uint8_t cheaterBuffer[72];
            memcpy(&cheaterBuffer[1], usb->rxBuffer, usb->bytesReceived);
            *(uint16_t *)&cheaterBuffer[0] = drive.getId();
            cheaterBuffer[2] = 16; // Cheater buffer is a dirty trick to make USB_FRAME_MD80_GENERIC_FRAME response compatibile with __updateResponseData
            drive.__updateResponseData((mab::StdMd80ResponseFrame_t *)cheaterBuffer);
        }
    }

    bool Candle::addMd80(uint16_t canId, MotorCommand_T config, bool printFailure)
    {
        if (inUpdateMode())
            return false;
        
        if (md80s.count(canId))
        {
            vout << "Md80 with ID: " << canId << " is already on the update list." << statusOK << std::endl;
            return true;
        }

        if ((int)md80s.size() >= maxDevices)
        {
            vout << "Cannot add more drives in current FAST_MODE. Max devices in current mode: " << maxDevices << statusFAIL << std::endl;
            return false;
        }

        AddMd80Frame_t add = {USB_FRAME_MD80_ADD, canId};
        if (usb->transmit((char *)&add, sizeof(AddMd80Frame_t), true))
            if (usb->rxBuffer[0] == USB_FRAME_MD80_ADD)
                if (usb->rxBuffer[1] == true)
                {
                    vout << "Added Md80." << statusOK << std::endl;
                    md80s.insert(std::pair<int, Md80>(canId, Md80(canId, config)));
                    md80Ids.push_back(canId);
                    mab::Md80 &newDrive = md80s.at(canId);
                    sendGetInfoFrame(newDrive);
                    sendMotionCommand(newDrive, newDrive.getPosition(), 0.0f, 0.0f);
                    newDrive.setTargetPosition(newDrive.getPosition());
                    return true;
                }
        if (printFailure)
            vout << "Failed to add Md80." << statusFAIL << std::endl;
        return false;
    }

    std::vector<uint16_t> Candle::ping(mab::CANdleBaudrate_E baudrate)
    {
        if (!this->configCandleBaudrate(baudrate))
            return std::vector<uint16_t>();
        vout << "Starting pinging drives at baudrate: " << baudrate << "M" << std::endl;
        char tx[128];
        tx[0] = USB_FRAME_PING_START;
        tx[1] = 0x00;
        std::vector<uint16_t> ids;
        if (usb->transmit(tx, 2, true, 2500)) // Scanning 2047 FDCAN ids, takes ~2100ms, thus wait for 2.5 sec
        {
            uint16_t *idsPointer = (uint16_t *)&usb->rxBuffer[1];
            for (int i = 0; i < 12; i++)
            {
                uint16_t id = idsPointer[i];
                if (id == 0x00)
                    break;
                ids.push_back(id);
            }
            if (ids.size() == 0)
            {
                vout << "No drives found." << std::endl;
                return ids;
            }
            vout << "Found drives." << std::endl;
            for (size_t i = 0; i < ids.size(); i++)
            {
                if (ids[i] == 0)
                    break; // No more ids in the message

                vout << std::to_string(i + 1) << ": ID = " << ids[i] << " (0x" << std::hex << ids[i] << std::dec << ")" << std::endl;
                if (ids[i] > 2047)
                {
                    vout << "Error! This ID is invalid! Probably two or more drives share same ID."
                         << "Communication will most likely be broken until IDs are unique!" << statusFAIL << std::endl;
                    std::vector<uint16_t> empty;
                    return empty;
                }
            }
        }
        return ids;
    }

    std::vector<uint16_t> Candle::ping()
    {
        return ping(mab::CANdleBaudrate_E::CAN_BAUD_1M);
    }
    
    bool Candle::sendGenericFDCanFrame(uint16_t canId, int msgLen, const char *txBuffer, char *rxBuffer, int timeoutMs)
    {
        int fdcanTimeout = timeoutMs - 3;
        if (timeoutMs < 3)
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
        if (usb->transmit(tx, len, true, timeoutMs)) // Got some response
        {
            if (usb->rxBuffer[0] == tx[0] && // USB Frame ID matches
                usb->rxBuffer[1] == true &&
                usb->bytesReceived <= 64 + 2) // response can ID matches
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
        *(uint16_t *)&frame.canMsg[2] = newId;
        *(uint32_t *)&frame.canMsg[4] = newBaudrateMbps * 1000000;
        *(uint16_t *)&frame.canMsg[8] = newTimeout;
        char tx[63];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if (usb->transmit(tx, len, true, 100))
            if (usb->rxBuffer[1] == 1)
            {
                vout << "CAN config change successful!" << statusOK << std::endl;
                vout << "Drive ID = " << std::to_string(canId) << " was changed to ID = " << std::to_string(newId) << std::endl;
                vout << "It's baudrate is now " << std::to_string(newBaudrateMbps) << "Mbps" << std::endl;
                vout << "It's CAN timeout (watchdog) is now " << (newTimeout == 0 ? "Disabled" : std::to_string(newTimeout) + "ms") << std::endl;
                return true;
            }
        vout << "CAN config change failed!" << statusFAIL << std::endl;
        return false;
    }
   
    bool Candle::configMd80Save(uint16_t canId)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_CAN_SAVE);
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if (usb->transmit(tx, len, true, 500))
            if (usb->rxBuffer[1] == true)
            {
                vout << "Saving in flash successful at ID = " << canId << statusOK << std::endl;
                return true;
            }
        vout << "Saving in flash failed at ID = " << canId << statusFAIL << std::endl;
        return false;
    }
 
    bool Candle::configMd80Blink(uint16_t canId)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_FLASH_LED);
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if (usb->transmit(tx, len, true, 500))
            if (usb->rxBuffer[1] == true)
            {
                vout << "LEDs blining at ID = " << canId << statusOK << std::endl;
                return true;
            }
        vout << "Blinking failed at ID = " << canId << statusFAIL << std::endl;
        return false;
    }

    bool Candle::controlMd80SetEncoderZero(uint16_t canId)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_ZERO_ENCODER);
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if (usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {
                /* set target position to 0.0f to avoid jerk at startup */
                Md80 &drive = md80s.at(canId);
                sendMotionCommand(drive, 0.0f, 0.0f, 0.0f);
                drive.setTargetPosition(0.0f);
                vout << "Setting new zero position successful at ID = " << canId << statusOK << std::endl;
                return true;
            }
        vout << "Setting new zero position failed at ID = " << canId << statusFAIL << std::endl;
        return false;
    }
 
    bool Candle::configMd80SetCurrentLimit(uint16_t canId, float currentLimit)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 6, Md80FrameId_E::FRAME_BASE_CONFIG);
        *(float *)&frame.canMsg[2] = currentLimit;
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if (usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[0] == USB_FRAME_MD80_GENERIC_FRAME && usb->rxBuffer[1] == true)
            {
                vout << "Setting new current limit successful at ID = " << canId << statusOK << std::endl;
                return true;
            }
        vout << "Setting new current limit failed at ID = " << canId << statusFAIL << std::endl;
        return false;
    }

    bool Candle::configCandleBaudrate(CANdleBaudrate_E canBaudrate, bool printVersionInfo)
    {
        char tx[10];
        tx[0] = USB_FARME_CANDLE_CONFIG_BAUDRATE;
        tx[1] = (uint8_t)canBaudrate;
        if (usb->transmit(tx, 2, true, 50))
            if (usb->rxBuffer[0] == USB_FARME_CANDLE_CONFIG_BAUDRATE && usb->rxBuffer[1] == true)
            {
                candleDeviceVersion = usb->rxBuffer[2];
                if (printVersionInfo)
                {
                    vout << "Device firmware version: v" << candleDeviceVersion / 10 << "." << candleDeviceVersion % 10 << std::endl;
                    if (candleDeviceVersion < 14)
                        std::cout << "Your CANdle firmware seems to be out-dated. Contact MAB: support@mabrobotics.pl , for intructions how to update." << std::endl;
                }
                return true;
            }
        return false;
    }

    bool Candle::controlMd80SetEncoderZero(Md80 &drive)
    {
        return this->controlMd80SetEncoderZero(drive.getId());
    }
  
    bool Candle::controlMd80Enable(Md80 &drive, bool enable)
    {
        return this->controlMd80Enable(drive.getId(), enable);
    }
  
    bool Candle::controlMd80Mode(Md80 &drive, Md80Mode_E mode)
    {
        return this->controlMd80Mode(drive.getId(), mode);
    }
  
    bool Candle::controlMd80Mode(uint16_t canId, Md80Mode_E mode)
    {
        try
        {
            Md80 &drive = md80s.at(canId);
            GenericMd80Frame32 frame = _packMd80Frame(canId, 3, Md80FrameId_E::FRAME_CONTROL_SELECT);
            frame.canMsg[2] = mode;
            char tx[64];
            int len = sizeof(frame);
            memcpy(tx, &frame, len);
            if (usb->transmit(tx, len, true, 50))
                if (usb->rxBuffer[1] == true)
                {
                    vout << "Setting control mode successful at ID = " << canId << statusOK << std::endl;
                    drive.__setControlMode(mode);
                    return true;
                }
            vout << "Setting control mode failed at ID = " << canId << statusFAIL << std::endl;
            return false;
        }
        catch (const char *msg)
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
            if (usb->transmit(tx, len, true, 50))
                if (usb->rxBuffer[1] == true)
                {
                    if (enable)
                        vout << "Enabling successful at ID = " << canId << statusOK << std::endl;
                    else
                    {
                        vout << "Disabling successful at ID = " << canId << statusOK << std::endl;
                        this->md80s.at(canId).__updateRegulatorsAdjusted(false); // Drive will operate at default params
                    }
                    return true;
                }
            vout << "Enabling/Disabling failed at ID = " << canId << statusFAIL << std::endl;
            return false;
        }
        catch (const char *msg)
        {
            vout << msg << std::endl;
            return false;
        }
    }
  
    bool Candle::begin()
    {
        if (mode == CANdleMode_E::UPDATE)
        {
            vout << "Cannot run 'begin', already in update mode." << statusFAIL << std::endl;
            return false;
        }
        char tx[128];
        tx[0] = USB_FRAME_BEGIN;
        tx[1] = 0x00;
        if (usb->transmit(tx, 2, true, 10))
        {
            vout << "Beginnig auto update loop mode" << statusOK << std::endl;
            if (_useLogs)
            {
                std::string homedir = getenv("HOME");
                std::string receiveFileName = homedir + "/log/latest/candle_receive" + std::to_string(candleId) + ".csv";
                std::string transmitFileName = homedir + "/log/latest/candle_transmit" + std::to_string(candleId) + ".csv";

                vout << "Candle" << candleId << "receive log file is: " << receiveFileName << std::endl;
                vout << "Candle" << candleId << "transmit log file is: " << transmitFileName << std::endl;

                receiveLogFile.open(receiveFileName, std::fstream::out);
                receiveLogFile << "frame_id, time, list[poisiton velocity torque temperature]" << std::endl;
                transmitLogFile.open(transmitFileName, std::fstream::out);
                transmitLogFile <<"frame_id, time, list[target_poisiton target_velocity target_torque kp kd position velocity effort]" << std::endl;
            }
            mode = CANdleMode_E::UPDATE;
            shouldStopTransmitter = false;
            shouldStopReceiver = false;
            msgsSent = 0;
            msgsReceived = 0;
            transmitterThread = std::thread(&Candle::transmit, this);
            receiverThread = std::thread(&Candle::receive, this);
            return true;
        }
        vout << "Failed to begin auto update loop mode" << statusFAIL << std::endl;
        return false;
    }
  
    bool Candle::end()
    {
        if (mode == CANdleMode_E::CONFIG)
            return false;

        shouldStopReceiver = true;
        if (receiverThread.joinable())
            receiverThread.join();

        shouldStopTransmitter = true;
        if (transmitterThread.joinable())
            transmitterThread.join();

        char tx[128];
        tx[0] = USB_FRAME_END;
        tx[1] = 0x00;
        usb->transmit(tx, 2, true, 10); // Stops update but produces garbage output

        if (usb->transmit(tx, 2, true, 10))
            if (usb->rxBuffer[0] == USB_FRAME_END && usb->rxBuffer[1] == 1)
                mode = CANdleMode_E::CONFIG;

        vout << "Ending auto update loop mode" << (mode == CANdleMode_E::CONFIG ? statusOK : statusFAIL) << std::endl;

        return mode == CANdleMode_E::CONFIG ? true : false;
    }
  
    bool Candle::reset()
    {
        char tx[128];
        tx[0] = USB_FRAME_RESET;
        tx[1] = 0x00;
        if (usb->transmit(tx, 2, true, 100))
            return true;

        return false;
    }

    bool Candle::inUpdateMode()
    {
        if (mode == CANdleMode_E::UPDATE)
            return true;
        return false;
    }

    bool Candle::inConfigMode()
    {
        if (mode == CANdleMode_E::CONFIG)
            return true;
        return false;
    }

    void Candle::transmitNewStdFrame()
    {
        char tx[512];
        tx[0] = USB_FRAME_UPDATE;
        std::vector<int> frameIds;
        int i = 0;
        std::string output = "";
        for (auto &[canId, md80Drive] : md80s)
        {
            md80Drive.__updateCommandFrame();
            output += "," + std::to_string(canId) + ":" +
            std::to_string(md80Drive.getTargetPos()) + " " + 
            std::to_string(md80Drive.getTargetVel()) + " " +
            std::to_string(md80Drive.getTorqueRequest()) + " " +
            std::to_string(md80Drive.getKP())+ " " +
            std::to_string(md80Drive.getKD())+ " " +
            std::to_string(md80Drive.getPosition()) + " " +
            std::to_string(md80Drive.getVelocity()) + " " +
            std::to_string(md80Drive.getTorque()) + " ";
            frameIds.push_back(md80Drive.getFrameId());
            *(StdMd80CommandFrame_t *)&tx[1 + i * sizeof(StdMd80CommandFrame_t)] = md80Drive.__getCommandFrame();
            i++;
        }

        int length = 1 + md80s.size() * sizeof(StdMd80CommandFrame_t);
        usb->transmit(tx, length, false, 100, candleId);
        uint64_t nsec = std::chrono::duration_cast<nsec_t>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (_useLogs)
        {
            for (const auto &e : frameIds)
                transmitLogFile << e << " ";

            double timeInSec = nsec * 1e-9;
            transmitLogFile << "," << std::to_string(timeInSec) << output <<std::endl;
        }
    }

    bool Candle::setupMd80Calibration(uint16_t canId, uint16_t torqueBandwidth)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 4, Md80FrameId_E::FRAME_CALIBRATION);
        char tx[64];
        frame.canMsg[2] = (uint8_t)(torqueBandwidth & 0xff);
        frame.canMsg[3] = (uint8_t)(torqueBandwidth >> 8);
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if (usb->transmit(tx, len, true, 50))
            if (usb->rxBuffer[1] == true)
            {
                vout << "Starting calibration at ID = " << canId << statusOK << std::endl;
                return true;
            }
        vout << "Starting calibration failed at ID = " << canId << statusFAIL << std::endl;
        return false;
    }

    bool Candle::setupMd80Diagnostic(uint16_t canId)
    {
        GenericMd80Frame32 frame = _packMd80Frame(canId, 2, Md80FrameId_E::FRAME_DIAGNOSTIC);
        char tx[64];
        int len = sizeof(frame);
        memcpy(tx, &frame, len);
        if (usb->transmit(tx, len, true, 50))
        {
            std::cout << "[CANDLE] Library version: " << getVersion() << std::endl;
            std::cout << "[CANDLE] DIAG at ID = " << canId << ": " << std::string(&usb->rxBuffer[2]) << std::endl;
            return true;
        }
        vout << "Diagnostic failed at ID = " << canId << std::endl;
        return false;
    }
}
