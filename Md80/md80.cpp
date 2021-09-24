#include "md80.hpp"

#include <iostream>
#include <unistd.h>
#include <chrono>

namespace mab
{
    //Md80 static variable definitions
    int Md80::numOfDrives = 0;
    Candle *Md80::pCan = nullptr;
    int Md80::commsFrequency;
    std::mutex Md80::commsMutex;
    std::thread Md80::commsThread;
    bool Md80::shouldStop = true;
    bool Md80::autoLoopEnabled = false;
    std::queue<Md80::Msg> Md80::msgQueue;
    std::list<Md80*> Md80::mdList;

    const int stdResponseLen = 16;

    int _getCurrentTimestamp()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

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
    float Md80::getPosition() {return position;}
    float Md80::getVelocity() {return velocity;}
    float Md80::getTorque() {return torque;}
    int Md80::getId() {return id;}

    void Md80::enableAutoLoop()
    {   
        autoLoopEnabled = true;    
    }
    void Md80::disableAutoLoop()
    {
        autoLoopEnabled = false; 
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
        if(numOfDrives == 0 || autoLoopEnabled == false)
            return;
        int oldestTimestamp = INT32_MAX;
        mab::Md80 * oldestDrive;
        auto drive = mdList.front();
        for (int i = 0 ; i < numOfDrives; i++)
        {
            if (drive->lastTimestamp < oldestTimestamp)
            {
                oldestTimestamp = drive->lastTimestamp;
                oldestDrive = drive;
            }
            std::advance(drive, 1);
        }
        commsMutex.lock();
        oldestDrive->sendGetInfo();
        commsMutex.unlock();
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

    bool Md80::flashLed()
    {
        return sendFlashLed();
    }
    bool Md80::enable()
    {
        return sendEnableMotor(true);
    }
    bool Md80::disable()
    {
        return sendEnableMotor(false);
    }
    bool Md80::setControlMode(mab::Mode mode)
    {
        return sendMode(mode);
    }
    bool Md80::setZeroPosition()
    {
        return sendZeroPosition();
    }
    bool Md80::setCurrentLimit(float currentLimit)
    {
        return sendCurrentLimit(currentLimit);
    }
    bool Md80::setImpedanceController(float kp, float kd, float positionTarget, float velocityTarget, float torque, float maxOutput)
    {
        return sendImpedance(kp, kd, positionTarget, velocityTarget, torque, maxOutput);
    }
    bool Md80::setPositionController(float kp, float ki, float kd, float iWindup, float maxOutput, float positionTarget)
    {
        return sendPosition(kp, ki, kd, iWindup, positionTarget, maxOutput);
    }
    bool Md80::setVelocityController(float kp, float ki, float kd, float iWindup, float maxOutput, float velocityTarget)
    {
        return sendVelocity(kp, ki, kd, iWindup, velocityTarget, maxOutput);
    }

    bool Md80::restart()
    {
        return sendReset();
    }

    bool Md80::configSetNewCanConfig(uint16_t newId, uint32_t newBaudrate)
    {
        if(newBaudrate != 1000000 && newBaudrate != 2500000 && newBaudrate != 4000000 && newBaudrate != 5000000 && newBaudrate != 2000000)
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
        Md80::commsMutex.unlock();
        return true;
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