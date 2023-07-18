#include "md80.hpp"

#include <string.h>

#include <iostream>

#include "mab_types.hpp"

namespace mab
{
void packImpedanceFrame(CanFrame_t* frame, RegImpedance_t* reg);
void packPidFrame(CanFrame_t* frame, RegPid_t* reg);

Md80::Md80(uint16_t _canID)
{
	canId = _canID;
	commandFrame.canId = _canID;
	memset(&regRead, 0, sizeof(regRead));
	memset(&regWrite, 0, sizeof(regWrite));
}
Md80::~Md80()
{
}
void Md80::setPositionControllerParams(float kp, float ki, float kd, float iWindup)
{
	regulatorsAdjusted = true;
	positionController.kp = kp;
	positionController.ki = ki;
	positionController.kd = kd;
	positionController.i_windup = iWindup;
}
void Md80::setVelocityControllerParams(float kp, float ki, float kd, float iWindup)
{
	velocityRegulatorAdjusted = true;
	velocityController.kp = kp;
	velocityController.ki = ki;
	velocityController.kd = kd;
	velocityController.i_windup = iWindup;
}
void Md80::setImpedanceControllerParams(float kp, float kd)
{
	regulatorsAdjusted = true;
	impedanceController.kp = kp;
	impedanceController.kd = kd;
}
void Md80::__updateCommandFrame()
{
	txCallback();

	switch (controlMode)
	{
		case Md80Mode_E::IDLE:
			commandFrame.toMd80.length = 2;
			commandFrame.toMd80.data[0] = Md80FrameId_E::FRAME_GET_INFO;
			commandFrame.toMd80.data[1] = 0;
			break;
		case Md80Mode_E::IMPEDANCE:
		case Md80Mode_E::RAW_TORQUE:
			if (regulatorsAdjusted)
			{
				packImpedanceFrame();
				regulatorsAdjusted = false;
			}
			else
				packMotionTargetsFrame();
			break;
		case Md80Mode_E::POSITION_PID:
		case Md80Mode_E::POSITION_PROFILE:
			if (regulatorsAdjusted)
			{
				packPositionFrame();
				regulatorsAdjusted = false;
			}
			else if (velocityRegulatorAdjusted)
			{
				packVelocityFrame();
				velocityRegulatorAdjusted = false;
			}
			else
				packMotionTargetsFrame();
			break;
		case Md80Mode_E::VELOCITY_PID:
		case Md80Mode_E::VELOCITY_PROFILE:
			if (velocityRegulatorAdjusted)
			{
				packVelocityFrame();
				velocityRegulatorAdjusted = false;
			}
			else
				packMotionTargetsFrame();
			break;
		default:
			break;
	}
}
void Md80::__updateResponseData(StdMd80ResponseFrame_t* _responseFrame)
{
	CanFrame_t frame = _responseFrame->fromMd80;

	if (_responseFrame->canId != canId || frame.data[0] != Md80FrameId_E::RESPONSE_DEFAULT)
		return;
	errorVector = *(uint16_t*)&frame.data[1];
	temperature = frame.data[3];
	position = *(float*)&frame.data[4];
	velocity = *(float*)&frame.data[8];
	torque = *(float*)&frame.data[12];
	outputEncoderPosition = *(float*)&frame.data[16];
	outputEncoderVelocity = *(float*)&frame.data[20];

	if (controlMode == POSITION_PID || controlMode == POSITION_PROFILE)
		targetPositionReached = static_cast<bool>(errorVector & 0x8000);
	else if (controlMode == VELOCITY_PID || controlMode == VELOCITY_PROFILE)
		targetVelocityReached = static_cast<bool>(errorVector & 0x8000);

	rxCallback();
}
void Md80::__updateRegulatorsAdjusted(bool adjusted)
{
	this->regulatorsAdjusted = adjusted;
}

void Md80::setMaxTorque(float maxTorque)
{
	this->maxTorque = maxTorque;
	maxTorqueAdjusted = true;
}
void Md80::setProfileVelocity(float profileVelocity)
{
	this->profileVelocity = profileVelocity;
	profileVelocityAdjusted = true;
}
void Md80::setProfileAcceleration(float profileAcceleration)
{
	this->profileAcceleration = profileAcceleration;
	profileAccelerationAdjusted = true;
}
void Md80::__setControlMode(Md80Mode_E mode)
{
	controlMode = mode;
}

void Md80::packImpedanceFrame()
{
	commandFrame.toMd80.length = 32;
	commandFrame.toMd80.data[0] = mab::Md80FrameId_E::FRAME_IMP_CONTROL;
	commandFrame.toMd80.data[1] = 0x00;
	*(float*)&commandFrame.toMd80.data[2] = impedanceController.kp;
	*(float*)&commandFrame.toMd80.data[6] = impedanceController.kd;
	*(float*)&commandFrame.toMd80.data[10] = positionTarget;
	*(float*)&commandFrame.toMd80.data[14] = velocityTarget;
	*(float*)&commandFrame.toMd80.data[18] = torqueTarget;
	*(float*)&commandFrame.toMd80.data[22] = maxTorque;
}
void Md80::packPositionFrame()
{
	commandFrame.toMd80.length = 32;
	commandFrame.toMd80.data[0] = mab::Md80FrameId_E::FRAME_POS_CONTROL;
	commandFrame.toMd80.data[1] = 0x00;
	*(float*)&commandFrame.toMd80.data[2] = positionController.kp;
	*(float*)&commandFrame.toMd80.data[6] = positionController.ki;
	*(float*)&commandFrame.toMd80.data[10] = positionController.kd;
	*(float*)&commandFrame.toMd80.data[14] = positionController.i_windup;
	*(float*)&commandFrame.toMd80.data[18] = profileVelocity;
	*(float*)&commandFrame.toMd80.data[22] = positionTarget;
}
void Md80::packVelocityFrame()
{
	commandFrame.toMd80.length = 32;
	commandFrame.toMd80.data[0] = mab::Md80FrameId_E::FRAME_VEL_CONTROL;
	commandFrame.toMd80.data[1] = 0x00;
	*(float*)&commandFrame.toMd80.data[2] = velocityController.kp;
	*(float*)&commandFrame.toMd80.data[6] = velocityController.ki;
	*(float*)&commandFrame.toMd80.data[10] = velocityController.kd;
	*(float*)&commandFrame.toMd80.data[14] = velocityController.i_windup;
	*(float*)&commandFrame.toMd80.data[18] = maxTorque;
	*(float*)&commandFrame.toMd80.data[22] = velocityTarget;
}
void Md80::packMotionTargetsFrame()
{
	commandFrame.toMd80.length = 28;
	commandFrame.toMd80.data[0] = mab::Md80FrameId_E::FRAME_SET_MOTION_TARGETS;
	commandFrame.toMd80.data[1] = 0x00;
	*(float*)&commandFrame.toMd80.data[2] = velocityTarget;
	*(float*)&commandFrame.toMd80.data[6] = positionTarget;
	*(float*)&commandFrame.toMd80.data[10] = torqueTarget;
	*(float*)&commandFrame.toMd80.data[14] = maxTorqueAdjusted ? maxTorque : NAN;
	*(float*)&commandFrame.toMd80.data[18] = profileVelocityAdjusted ? profileVelocity : NAN;
	*(float*)&commandFrame.toMd80.data[22] = profileAccelerationAdjusted ? profileAcceleration : NAN;
}

}  // namespace mab