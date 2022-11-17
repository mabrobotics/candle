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
	regulatorsAdjusted = true;
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
	switch (controlMode)
	{
		case Md80Mode_E::IDLE:
			commandFrame.toMd80.length = 2;
			commandFrame.toMd80.data[0] = Md80FrameId_E::FRAME_GET_INFO;
			commandFrame.toMd80.data[1] = 0;
			break;
		case Md80Mode_E::IMPEDANCE:
			if (regulatorsAdjusted)
				packImpedanceFrame();
			else
				packMotionTargetsFrame();
			break;
		case Md80Mode_E::POSITION_PID:
			if (regulatorsAdjusted)
				if (velocityRegulatorAdjusted)
				{
					velocityRegulatorAdjusted = false;
					packVelocityFrame();
				}
				else
					packPositionFrame();
			else
				packMotionTargetsFrame();
			break;
		case Md80Mode_E::VELOCITY_PID:
			if (regulatorsAdjusted)
				packVelocityFrame();
			else
				packMotionTargetsFrame();
			break;
		default:
			break;
	}
	/* send updated gains only when modified */
	regulatorsAdjusted = false;
}
void Md80::__updateResponseData(StdMd80ResponseFrame_t* _responseFrame)
{
	if (_responseFrame->canId != canId || _responseFrame->fromMd80.data[0] != Md80FrameId_E::RESPONSE_DEFAULT)
		return;
	errorVector = *(uint16_t*)&_responseFrame->fromMd80.data[1];
	temperature = _responseFrame->fromMd80.data[3];
	position = *(float*)&_responseFrame->fromMd80.data[4];
	velocity = *(float*)&_responseFrame->fromMd80.data[8];
	torque = *(float*)&_responseFrame->fromMd80.data[12];
}
void Md80::__updateRegulatorsAdjusted(bool adjusted)
{
	this->regulatorsAdjusted = adjusted;
}

// advanced setters
void Md80::setMaxTorque(float _maxTorque)
{
	maxTorque = _maxTorque;
}
void Md80::setMaxVelocity(float _maxVelocity)
{
	maxVelocity = _maxVelocity;
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
	*(float*)&commandFrame.toMd80.data[18] = torqueSet;
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
	*(float*)&commandFrame.toMd80.data[18] = maxVelocity;
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
	commandFrame.toMd80.length = 16;
	commandFrame.toMd80.data[0] = mab::Md80FrameId_E::FRAME_SET_MOTION_TARGETS;
	commandFrame.toMd80.data[1] = 0x00;
	*(float*)&commandFrame.toMd80.data[2] = velocityTarget;
	*(float*)&commandFrame.toMd80.data[6] = positionTarget;
	*(float*)&commandFrame.toMd80.data[10] = torqueSet;
}

uint16_t getRegisterSize(uint16_t regId)
{
	switch (regId)
	{
		case Md80Reg_E::bridgeType:
		case Md80Reg_E::outputEncoder:
		case Md80Reg_E::hardwareVersion:
		case Md80Reg_E::canTermination:
			return 1;
		case Md80Reg_E::motorTorgueBandwidth:
		case Md80Reg_E::canWatchdog:
		case Md80Reg_E::errorVector:
		case Md80Reg_E::motorKV:
			return 2;
		case Md80Reg_E::temperature:
		case Md80Reg_E::temperatureAux:
		case Md80Reg_E::motorInductance:
		case Md80Reg_E::motorResistance:
		case Md80Reg_E::firmwareVersion:
		case Md80Reg_E::buildDate:
		case Md80Reg_E::motorImpPidKp:
		case Md80Reg_E::motorImpPidKd:
		case Md80Reg_E::motorImpPidOutMax:
		case Md80Reg_E::motorPosPidKp:
		case Md80Reg_E::motorPosPidKi:
		case Md80Reg_E::motorPosPidKd:
		case Md80Reg_E::motorPosPidOutMax:
		case Md80Reg_E::motorPosPidWindup:
		case Md80Reg_E::motorVelPidKp:
		case Md80Reg_E::motorVelPidKi:
		case Md80Reg_E::motorVelPidKd:
		case Md80Reg_E::motorVelPidOutMax:
		case Md80Reg_E::motorVelPidWindup:
		case Md80Reg_E::motorFriction:
		case Md80Reg_E::motorStiction:
		case Md80Reg_E::outputEncoderDir:
		case Md80Reg_E::outputEncoderDefaultBaud:
		case Md80Reg_E::canBaudrate:
		case Md80Reg_E::motorGearRatio:
		case Md80Reg_E::motorPolePairs:
		case Md80Reg_E::motorKt:
		case Md80Reg_E::motorKt_a:
		case Md80Reg_E::motorKt_b:
		case Md80Reg_E::motorKt_c:
		case Md80Reg_E::motorIMax:
			return 4;
		case Md80Reg_E::commitHash:
			return 8;
		case Md80Reg_E::motorName:
			return 24;
		default:
			return 0;
	}
}

}  // namespace mab