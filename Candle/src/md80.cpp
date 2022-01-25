#include "md80.hpp"
#include "mab_types.hpp"

namespace mab
{
    void packImpedanceFrame(CanFrame_t*frame, RegImpedance_t*reg);
    void packPidFrame(CanFrame_t*frame, RegPid_t*reg);

    Md80::Md80(uint16_t _canID)
    {
        canId = _canID;
        commandFrame.canId = _canID;
    }
    Md80::~Md80()
    {

    }
    void Md80::setPositionRegulator(float kp, float ki, float kd, float iWindup)
    {
        regulatorsAdjusted = true;
        positionRegulator.kp = kp;
        positionRegulator.ki = ki;
        positionRegulator.kd = kd;
        positionRegulator.i_windup = iWindup;
    }
    void Md80::setVelocityRegulator(float kp, float ki, float kd, float iWindup)
    {
        regulatorsAdjusted = true;
        velocityRegulator.kp = kp;
        velocityRegulator.ki = ki;
        velocityRegulator.kd = kd;
        velocityRegulator.i_windup = iWindup;
    }
    void Md80::setImpedanceRegulator(float kp, float kd)
    {
        regulatorsAdjusted = true;
        impedanceRegulator.kp = kp;
        impedanceRegulator.kd = kd;
    }
    void Md80::updateCommandFrame()
    {
        switch (controlMode)
        {
        case Md80Mode_E::IDLE:
            commandFrame.toMd80.length = 2;
            commandFrame.toMd80.data[0] = Md80FrameId_E::FRAME_GET_INFO;
            commandFrame.toMd80.data[1] = 0;
            break;
        case Md80Mode_E::IMPEDANCE:
            if(regulatorsAdjusted)
                packImpedanceFrame();
            else
                packMotionTargetsFrame();
            break;
        case Md80Mode_E::POSITION_PID:
            if(regulatorsAdjusted)
                packPositionFrame();
            else
                packMotionTargetsFrame();
            break;
        case Md80Mode_E::VELOCITY_PID:
            if(regulatorsAdjusted)
                packVelocityFrame();
            else
                packMotionTargetsFrame();
            break;
        case Md80Mode_E::TORQUE:
            packMotionTargetsFrame();
            break;
        default:
            break;
        }
    }
    void Md80::updateResponseData(StdMd80ResponseFrame_t*_responseFrame)
    {
        if(_responseFrame->canId != canId || _responseFrame->fromMd80.data[0] != Md80FrameId_E::RESPONSE_DEFAULT)
            return;
        errorVector = *(uint16_t*)&_responseFrame->fromMd80.data[1];
        position = *(float*)&_responseFrame->fromMd80.data[4];
        velocity = *(float*)&_responseFrame->fromMd80.data[8];
        torque = *(float*)&_responseFrame->fromMd80.data[12];
    }

    //advanced setters
    void Md80::setMaxTorque(float _maxTorque)
    {
        maxTorque = _maxTorque;
    }
    void Md80::setMaxVelocity(float _maxVelocity)
    {
        maxVelocity = _maxVelocity;
    }
    void Md80::setControlMode(Md80Mode_E mode)
    {
        controlMode = mode;
    }

    void Md80::packImpedanceFrame()
    {
        commandFrame.toMd80.length = 32;
        commandFrame.toMd80.data[0] = 0x12;
        commandFrame.toMd80.data[1] = 0x00;
        *(float*)&commandFrame.toMd80.data[2] = impedanceRegulator.kp;
        *(float*)&commandFrame.toMd80.data[6] = impedanceRegulator.kd;
        *(float*)&commandFrame.toMd80.data[10] = positionTarget;
        *(float*)&commandFrame.toMd80.data[14] = velocityTarget;
        *(float*)&commandFrame.toMd80.data[18] = torqueSet;
        *(float*)&commandFrame.toMd80.data[22] = maxTorque;
    }
    void Md80::packPositionFrame()
    {
        commandFrame.toMd80.length = 32;
        commandFrame.toMd80.data[0] = 0x10;
        commandFrame.toMd80.data[1] = 0x00;
        *(float*)&commandFrame.toMd80.data[2] = positionRegulator.kp;
        *(float*)&commandFrame.toMd80.data[6] = positionRegulator.ki;
        *(float*)&commandFrame.toMd80.data[10] = positionRegulator.kd;
        *(float*)&commandFrame.toMd80.data[14] = positionRegulator.i_windup;
        *(float*)&commandFrame.toMd80.data[18] = positionTarget;
        *(float*)&commandFrame.toMd80.data[22] = maxVelocity;
    }
    void Md80::packVelocityFrame()
    {
        commandFrame.toMd80.length = 32;
        commandFrame.toMd80.data[0] = 0x11;
        commandFrame.toMd80.data[1] = 0x00;
        *(float*)&commandFrame.toMd80.data[2] = velocityRegulator.kp;
        *(float*)&commandFrame.toMd80.data[6] = velocityRegulator.ki;
        *(float*)&commandFrame.toMd80.data[10] = velocityRegulator.kd;
        *(float*)&commandFrame.toMd80.data[14] = velocityRegulator.i_windup;
        *(float*)&commandFrame.toMd80.data[18] = velocityTarget;
        *(float*)&commandFrame.toMd80.data[22] = maxTorque;
    }
    void Md80::packMotionTargetsFrame()
    {
        commandFrame.toMd80.length = 16;
        commandFrame.toMd80.data[0] = 0x14;
        commandFrame.toMd80.data[1] = 0x00;
        *(float*)&commandFrame.toMd80.data[2] = velocityTarget;
        *(float*)&commandFrame.toMd80.data[6] = positionTarget;
        *(float*)&commandFrame.toMd80.data[10] = torqueSet;
    }
}