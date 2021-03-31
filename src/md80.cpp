#include "md80.hpp"
#include "canalizator.hpp"

#include <iostream>
#include "unistd.h"
Md80::Md80(Canalizator *can, int driveId)
{
    pCan = can;
    id = driveId;
    position = 0.0f;
    velocity = 0.0f;
    torque = 0.0f;
    errorVector = 0.0f;

    posTarget = 0.0f;
    velTarget= 0.0f;
    kp= 0.0f;
    kd= 0.0f;
    tqFf= 0.0f;
    maxOutput= 0.0f;
}
void Md80::parseResponse(char rxBuffer[])
{
    errorVector = *(uint16_t*)&rxBuffer[1];
    position = *(float*)&rxBuffer[4];
    velocity = *(float*)&rxBuffer[4+4];
    torque = *(float*)&rxBuffer[4+12];
}
bool Md80::enableMotor(bool enable)
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
    if(rxLen == 20)
    {
        parseResponse(rxBuffer);
        return true;
    }
    return false;
}
bool Md80::setMode(int mode)
{
    pCan->setTargetId(id);
    pCan->setMsgLen(3);
    char txBuffer[3] = {0x02, 0x00, (unsigned char)mode};
    char rxBuffer[64];
    pCan->setCanTx(txBuffer, 3);
    pCan->transmitAndReceive();
    int rxLen = pCan->getCanRx(rxBuffer);
    if(rxLen == 20)
    {
        parseResponse(rxBuffer);
        return true;
    }
    return false;
}
bool Md80::setImpedance()
{
    return setImpedance(kp,kd, posTarget, velTarget, torque, maxOutput);
}
bool Md80::setImpedance(float _kp, float _kd, float _posTarget, float _velTarget, float _torque, float _maxOutput)
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
    if(rxLen == 20)
    {
        parseResponse(rxBuffer);
        kp = _kp;
        kd = _kd;
        posTarget = _posTarget;
        velTarget = _velTarget;
        tqFf = _torque;
        maxOutput = _maxOutput;
        return true;
    }
    return false;
}

std::vector<int> Md80::sendPing(Canalizator*pCan, int idStart, int idEnd)
{
    std::vector<int> drivesPinged;
    pCan->setMsgLen(5);
    uint8_t buffer[5]; buffer[0] = 0x00; buffer[1] = 0x00; buffer[2] = 0x00;
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
    return drivesPinged;
}

void Md80::printInfo()
{
    std::cout << "Drive 0x" << std::hex << id << std::dec << " - Pos: " << position << ", Vel: " << velocity <<
        ", Torque: " << torque << std::endl;
}