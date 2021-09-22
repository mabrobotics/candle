#include "md80.hpp"

#include <iostream>
#include <unistd.h>

namespace mab
{
    //Md80 static variable definitions
    int Md80::numOfDrives = 0;
    Candle *Md80::pCan = nullptr;
    int Md80::commsFrequency;
    std::mutex Md80::commsMutex;
    std::thread Md80::commsThread;
    bool Md80::shouldStop = true;
    bool Md80::shouldPause = true;
    std::queue<Md80::Msg> Md80::msgQueue;
    std::list<Md80*> Md80::mdList;

    const int stdResponseLen = 16;

    Md80::Md80(int driveId)
    {
        if(Md80::pCan == nullptr)
        {
            std::cout << "Md80 not initialized! Use Md80::initialize before creating Md80 objects!" << std::endl;
            return;
        }
        Md80::mdList.push_back(this);
        Md80::numOfDrives++;
        id = driveId;
        position = 0.0f;
        velocity = 0.0f;
        torque = 0.0f;
        errorVector = 0.0f;
        driveOk = false;
    }
    Md80::~Md80()
    {
        mdList.remove(this);
        Md80::numOfDrives--;
    }
    void Md80::enableAutoLoop()
    {   
        shouldPause = false;    
    }
    void Md80::disableAutoLoop()
    {
           shouldPause = true; 
    }
    void Md80::_commsThreadCallback()
    {
        while(true)
        {
            if(shouldStop)
                return;

            _commsPerform();

            float period = 1.0f / (float)commsFrequency;
            int millis = period * 1000.0f;
            std::this_thread::sleep_for(std::chrono::milliseconds(millis));
        }
    }
    void Md80::_commsPerform()
    {
        if(numOfDrives == 0)
            return;     //No drives initialized so far - nothing to do
        
        //there are drives present 
        
        if(msgQueue.size() == 0 && shouldPause)
            return; //empty queue, and should not auto loop
            
        if(msgQueue.size() == 0)
        {
            // User haven't commanded anything - should send GetInfo for the next drive
            static int driveIterator = 0;
            if(driveIterator < (int)mdList.size())
            {
                auto item = mdList.begin();
                std::advance(item, driveIterator);
                msgQueue.push(Md80::Msg((*item)->id, MsgType::GET_INFO));
                if(++driveIterator >= (int)mdList.size())
                    driveIterator = 0;
            }
        }
        //There is users msg in queue
        Md80*drive; //Pointer to currently served drive
        for(auto md80 : mdList)
            if(md80->id == msgQueue.front().id)
                drive = md80;
        bool msgSuccess = false;
        switch (msgQueue.front().msgType)
        {
        case MsgType::FLASH_LED:
        {
            msgSuccess = drive->sendFlashLed();
            break;
        }
        case MsgType::ENABLE:
        {
            msgSuccess = drive->sendEnableMotor((uint8_t)msgQueue.front().data[0]);
            break;
        }
        case MsgType::CONTROL_SELECT:
        {
            msgSuccess = drive->sendMode((mab::Mode)msgQueue.front().data[0]);
            break;
        }
        case MsgType::SET_ZERO_POSITION:
        {
            msgSuccess = drive->sendZeroPosition();
            break;
        }
        case MsgType::SET_MAX_CURRENT:
        {
            msgSuccess = drive->sendCurrentLimit(*(float*)&msgQueue.front().data[0]);
            break;
        }
        case MsgType::SET_POSITION_REG:
        {
            msgSuccess = drive->sendPosition(*(float*)&msgQueue.front().data[0],
                                            *(float*)&msgQueue.front().data[4],
                                            *(float*)&msgQueue.front().data[8],
                                            *(float*)&msgQueue.front().data[12],
                                            *(float*)&msgQueue.front().data[16],
                                            *(float*)&msgQueue.front().data[20]);
            break;
        }
        case MsgType::SET_VELOCITY_REG:
        {
            msgSuccess = drive->sendVelocity(*(float*)&msgQueue.front().data[0],
                                            *(float*)&msgQueue.front().data[4],
                                            *(float*)&msgQueue.front().data[8],
                                            *(float*)&msgQueue.front().data[12],
                                            *(float*)&msgQueue.front().data[16],
                                            *(float*)&msgQueue.front().data[20]);
            break;
        }
        case MsgType::SET_IMPEDANCE_REG:
        {
            msgSuccess = drive->sendImpedance(*(float*)&msgQueue.front().data[0],
                                            *(float*)&msgQueue.front().data[4],
                                            *(float*)&msgQueue.front().data[8],
                                            *(float*)&msgQueue.front().data[12],
                                            *(float*)&msgQueue.front().data[16],
                                            *(float*)&msgQueue.front().data[20]);
            break;
        }
        case MsgType::GET_INFO:
        {
            msgSuccess = drive->sendGetInfo();
            break;
        }
        case MsgType::RESET_DRIVE:
        {
            msgSuccess = drive->sendReset();
            break;
        }
        default:
            break;
        }
        msgQueue.pop();
        drive->driveOk = msgSuccess;
    }
    void Md80::initalize(Candle *can)
    {
        pCan = can;
        pCan->setRxTimeout(3000);
        Md80::numOfDrives = 0;
        Md80::commsFrequency = 100;
        shouldStop = false;
        Md80::commsThread = std::thread(_commsThreadCallback);
    }
    void Md80::deinitalize()
    {
        shouldStop = true;
        Md80::commsThread.join();
    }
    void Md80::_parseResponse(char rxBuffer[])
    {
        errorVector = *(uint16_t*)&rxBuffer[1];
        position = *(float*)&rxBuffer[4];
        velocity = *(float*)&rxBuffer[4+4];
        torque = *(float*)&rxBuffer[4+8];
    }

    void Md80::flashLed()
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::FLASH_LED));
    }
    void Md80::enable()
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::ENABLE));
        Md80::msgQueue.back().data[0] = 1;
    }
    void Md80::disable()
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::ENABLE));
        Md80::msgQueue.back().data[0] = 0;
    }
    void Md80::setControlMode(mab::Mode mode)
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::CONTROL_SELECT));
        Md80::msgQueue.back().data[0] = (uint8_t)mode;
    }
    void Md80::setZeroPosition()
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::SET_ZERO_POSITION));
    }
    void Md80::setCurrentLimit(float currentLimit)
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::SET_MAX_CURRENT));
        *(float*)&Md80::msgQueue.back().data[0] = currentLimit;
    }
    void Md80::setImpedanceController(float kp, float kd, float positionTarget, float velocityTarget, float torque, float maxOutput)
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::SET_IMPEDANCE_REG));
        *(float*)&Md80::msgQueue.back().data[0] = kp;
        *(float*)&Md80::msgQueue.back().data[4] = kd;
        *(float*)&Md80::msgQueue.back().data[8] = positionTarget;
        *(float*)&Md80::msgQueue.back().data[12] = velocityTarget;
        *(float*)&Md80::msgQueue.back().data[16] = torque;
        *(float*)&Md80::msgQueue.back().data[20] = maxOutput;
    }
    void Md80::setPositionController(float kp, float ki, float kd, float iWindup, float maxOutput, float positionTarget)
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::SET_POSITION_REG));
        *(float*)&Md80::msgQueue.back().data[0] = kp;
        *(float*)&Md80::msgQueue.back().data[4] = ki;
        *(float*)&Md80::msgQueue.back().data[8] = kd;
        *(float*)&Md80::msgQueue.back().data[12] = iWindup;
        *(float*)&Md80::msgQueue.back().data[16] = maxOutput;
        *(float*)&Md80::msgQueue.back().data[20] = positionTarget;
    }
    void Md80::setVelocityController(float kp, float ki, float kd, float iWindup, float maxOutput, float velocityTarget)
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::SET_VELOCITY_REG));
        *(float*)&Md80::msgQueue.back().data[0] = kp;
        *(float*)&Md80::msgQueue.back().data[4] = ki;
        *(float*)&Md80::msgQueue.back().data[8] = kd;
        *(float*)&Md80::msgQueue.back().data[12] = iWindup;
        *(float*)&Md80::msgQueue.back().data[16] = maxOutput;
        *(float*)&Md80::msgQueue.back().data[20] = velocityTarget;
    }

    void Md80::restart()
    {
        Md80::msgQueue.push(Msg(this->id, Md80::MsgType::RESET_DRIVE));
    }

    bool Md80::configSetNewCanConfig(uint16_t newId, uint32_t newBaudrate)
    {
        if(newBaudrate != 1000000 && newBaudrate != 2500000 && newBaudrate != 5000000)
        {
            std::cout << "Selected baudrate [" << newBaudrate << "] is not supported!" << std::endl;
            return false;
        }
        if(newId > 0x07FF)
        {
            std::cout << "Selected CAN ID" << newId << " is out of range!" << std::endl;
            return false;
        }

        Md80::commsMutex.lock();
        pCan->setTargetId(id);
        pCan->setMsgLen(8);
        char txBuffer[6];
        char rxBuffer[64];
        txBuffer[0] = 0x20;
        *(uint16_t*)&txBuffer[2] = newId;
        *(uint32_t*)&txBuffer[4] = newBaudrate;
        pCan->setCanTx(txBuffer, 8);
        pCan->transmitAndReceive();
        pCan->getCanRx(rxBuffer);
        usleep(50000);
        pCan->setCanSpeed(newBaudrate);
        pCan->transmitAndReceive();
        pCan->getCanRx(rxBuffer);
        int oldId = this->id;
        this->id = newId;
        if(this->sendGetInfo())
            return true;
        this->id = oldId;
        Md80::commsMutex.unlock();
        return false;
    }
    bool Md80::configSaveNewConfig()
    {
        Md80::commsMutex.lock();
        pCan->setTargetId(id);
        pCan->setMsgLen(2);
        char txBuffer[2];
        char rxBuffer[64];
        txBuffer[0] = 0x21;
        pCan->setCanTx(txBuffer, 2);
        pCan->transmitAndReceive();
        pCan->getCanRx(rxBuffer);
        usleep(5000000);
        if(this->sendGetInfo())
            return true;
        Md80::commsMutex.unlock();
        return false;
    }

    bool Md80::configCalibration()
    {
        Md80::commsMutex.lock();
        pCan->setTargetId(id);
        pCan->setMsgLen(1);
        char txBuffer = {0x70};
        char rxBuffer[64];
        pCan->setCanTx(&txBuffer, 1);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);
        Md80::commsMutex.unlock();
        if(rxLen== stdResponseLen)
        {
            _parseResponse(rxBuffer);
            return true;
        }
        return false;
    }

    std::vector<int> Md80::sendPing(Candle*pCan, int idStart, int idEnd)
    {
        commsMutex.lock();
        std::vector<int> drivesPinged;
        pCan->setMsgLen(2);
        uint8_t buffer[2]; buffer[0] = 0x00; buffer[1] = 0x00;
        char rxBuffer[64];
        pCan->setCanTx((char*)buffer, 2);

        for(int i = idStart; i < idEnd; i++)
        {
            if (i == 0x69)
                continue;
            pCan->setTargetId(i);
            pCan->transmitAndReceive();
        
            int len = pCan->getCanRx(rxBuffer);
            if (len > 10)
                drivesPinged.push_back(i);
        }
        std::cout << "foundDrives: " << drivesPinged.size() << std::endl;
        for(int i = 0; i < (int)drivesPinged.size(); i++)
            std::cout << "[" << i << "]\tID: " << drivesPinged[i] << " (0x"<< std::hex << drivesPinged[i] << std::dec << ")" << std::endl; 
        commsMutex.unlock();
        return drivesPinged;
    }

    void Md80::printInfo()
    {
        std::cout << "Drive 0x" << std::hex << id << std::dec << " - Pos: " << position << ", Vel: " << velocity <<
            ", Torque: " << torque << ", Errors: "<< errorVector <<  std::endl;
    }
}