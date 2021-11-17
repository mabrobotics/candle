#include "md80.hpp"
namespace mab
{
    const int stdResponseLen = 16;
    bool Md80::sendFlashLed()
    {
        pCan->setTargetId(id);
        pCan->setMsgLen(2);
        char txBuffer[3] = {0x00, 0x00, 0x00};
        char rxBuffer[64];
        pCan->setCanTx(txBuffer, 3);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);

        if(rxLen== stdResponseLen)
        {
            _parseResponse(rxBuffer);
            return true;
        }
        return false;
    }
    bool Md80::sendEnableMotor(bool enable)
    {
        pCan->setTargetId(id);
        pCan->setMsgLen(3);
        char txBuffer[3] = {0x01, 0x00, 0x00};
        char rxBuffer[64];
        if(enable)
            txBuffer[2] = 0x01;
        pCan->setCanTx(txBuffer, 3);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);
        if(rxLen== stdResponseLen)
        {
            _parseResponse(rxBuffer);
            return true;
        }
        return false;
    }
    bool Md80::sendMode(Mode mode)
    {
        pCan->setTargetId(id);
        pCan->setMsgLen(3);
        char txBuffer[3] = {0x02, 0x00, (char)mode};
        char rxBuffer[64];
        pCan->setCanTx(txBuffer, 3);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);
        if(rxLen== stdResponseLen)
        {
            _parseResponse(rxBuffer);
            return true;
        }
        return false;
    }
    bool Md80::sendZeroPosition()
    {
        pCan->setTargetId(id);
        pCan->setMsgLen(2);
        char txBuffer[3] = {0x03, 0x00};
        char rxBuffer[64];
        pCan->setCanTx(txBuffer, 2);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);
        if(rxLen== stdResponseLen)
        {
            _parseResponse(rxBuffer);
            return true;
        }
        return false;
    }
    bool Md80::sendCurrentLimit(float newLimit)
    {
        pCan->setTargetId(id);
        pCan->setMsgLen(6);
        char txBuffer[6] = {0x04, 0x00};
        char rxBuffer[64];
        *(float*)&txBuffer[2] = newLimit;
        pCan->setCanTx(txBuffer, 6);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);
        if(rxLen== stdResponseLen)
        {
            currentLimit = newLimit;
            _parseResponse(rxBuffer);
            return true;
        }
        return false;
    }
    bool Md80::sendGetInfo()
    {
        Md80::commsMutex.lock();
        pCan->setTargetId(id);
        pCan->setMsgLen(2);
        char txBuffer[2] = {0x05, 0x00};
        char rxBuffer[64];
        pCan->setCanTx(txBuffer, 2);
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
    bool Md80::configStopWatchdog()
    {
        Md80::commsMutex.lock();
        pCan->setTargetId(id);
        pCan->setMsgLen(2);
        char txBuffer[6] = {0x19, 0x00};
        char rxBuffer[64];
        pCan->setCanTx(txBuffer, 2);
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

    bool Md80::sendImpedance(float _kp, float _kd, float _posTarget, float _velTarget, float _torque, float _maxOutput)
    {
        pCan->setTargetId(id);
        pCan->setMsgLen(32);
        char txBuffer[32];
        char rxBuffer[64];
        txBuffer[0] = 0x12;
        *(float*)&txBuffer[2] = _kp;
        *(float*)&txBuffer[2+4] = _kd;
        *(float*)&txBuffer[2+8] = _posTarget;
        *(float*)&txBuffer[2+12] = _velTarget;
        *(float*)&txBuffer[2+16] = _torque;
        *(float*)&txBuffer[2+20] = _maxOutput;
        pCan->setCanTx(txBuffer, 32);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);
        if(rxLen== stdResponseLen)
        {
            _parseResponse(rxBuffer);
            impedanceReg.kp = _kp;
            impedanceReg.kd = _kd;
            impedanceReg.posTarget = _posTarget;
            impedanceReg.velTarget = _velTarget;
            impedanceReg.torqueCmd = _torque;
            impedanceReg.maxOutput = _maxOutput;
            return true;
        }
        return false;
    }
    bool Md80::sendPosition(float kp, float ki, float kd, float ki_windup, float posTarget, float maxOutput)
    {
        pCan->setTargetId(id);
        pCan->setMsgLen(32);
        char txBuffer[32];
        char rxBuffer[64];
        txBuffer[0] = 0x10;
        *(float*)&txBuffer[2] = kp;
        *(float*)&txBuffer[2+4] = ki;
        *(float*)&txBuffer[2+8] = kd;
        *(float*)&txBuffer[2+12] = ki_windup;
        *(float*)&txBuffer[2+16] = maxOutput;
        *(float*)&txBuffer[2+20] = posTarget;
        pCan->setCanTx(txBuffer, 32);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);
        if(rxLen== stdResponseLen)
        {
            _parseResponse(rxBuffer);
            positionReg.kp = kp;
            positionReg.ki = ki;
            positionReg.kd = kd;
            positionReg.iWindup = ki_windup;
            positionReg.maxOutput = maxOutput;
            positionReg.posTarget = posTarget;
            return true;
        }
        return false;
    }
    bool Md80::sendVelocity(float kp, float ki, float kd, float ki_windup, float velTarget, float maxOutput)
    {
        pCan->setTargetId(id);
        pCan->setMsgLen(32);
        char txBuffer[32];
        char rxBuffer[64];
        txBuffer[0] = 0x11;
        *(float*)&txBuffer[2] = kp;
        *(float*)&txBuffer[2+4] = ki;
        *(float*)&txBuffer[2+8] = kd;
        *(float*)&txBuffer[2+12] = ki_windup;
        *(float*)&txBuffer[2+16] = maxOutput;
        *(float*)&txBuffer[2+20] = velTarget;
        pCan->setCanTx(txBuffer, 32);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);
        if(rxLen== stdResponseLen)
        {
            _parseResponse(rxBuffer);
            velocityReg.kp = kp;
            velocityReg.ki = ki;
            velocityReg.kd = kd;
            velocityReg.iWindup = ki_windup;
            velocityReg.maxOutput = maxOutput;
            velocityReg.posTarget = velTarget;
            return true;
        }
        return false;
    }
    bool Md80::sendReset()
    {
        pCan->setTargetId(id);
        pCan->setMsgLen(2);
        char txBuffer[2] = {0x13, 0x00};
        char rxBuffer[64];
        pCan->setCanTx(txBuffer, 2);
        pCan->transmitAndReceive();
        int rxLen = pCan->getCanRx(rxBuffer);
        if(rxLen== stdResponseLen)
        {
            _parseResponse(rxBuffer);
            return true;
        }
        return false;
    }
}