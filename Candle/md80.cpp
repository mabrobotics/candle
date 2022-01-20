#include "md80.hpp"

Md80::Md80(uint16_t _canID)
{
    canId = _canID;
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

}
void Md80::updateResponseData()
{

}

//advanced setters
void Md80::setMaxTorque(float maxTorque)
{

}
void Md80::setMaxVelocity(float maxVelocity)
{

}
void Md80::setControlMode(Md80Mode_E mode)
{

}