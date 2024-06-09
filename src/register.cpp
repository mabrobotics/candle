
#include "register.hpp"

#include "candle.hpp"

namespace mab
{

bool Register::prepare(uint16_t canId, mab::Md80FrameId_E frameType)
{
	(void)frameType;
	/* clear the RX buffer and send register request */
	memset(regRxBuffer, 0, sizeof(regRxBuffer));
	bool status = candle->sendGenericFDCanFrame(canId, regTxPtr - regTxBuffer, regTxBuffer, regRxBuffer, 10);
	regTxPtr = nullptr;
	regRxPtr = nullptr;
	return status;
}

bool Register::interpret(uint16_t canId)
{
	(void)canId;
	return true;
}

uint32_t Register::pack(uint16_t regId, char* value, char* buffer)
{
	uint32_t len = getSize(regId);
	/* in case no place is left in the buffer */
	if ((len + 2) > (sizeof(regTxBuffer) - (buffer - regTxBuffer)))
	{
		throw "Error while packaging data. Make sure its size is not above 62 bytes. Remember to add 2 bytes per field (field ID).";
		return 0;
	}
	/* place register ID at the beginning */
	*(uint16_t*)buffer = regId;
	/* move the pointer forward by 2 bytes */
	buffer += sizeof(regId);
	/* in case we're just preparing a read frame */
	if (value == nullptr)
		memset(buffer, 0, len);
	else
		memcpy(buffer, value, len);

	return (len + sizeof(regId));
}

uint32_t Register::unPack(uint16_t regId, char* value, char* buffer)
{
	/* place register ID at the beginning */
	*(uint16_t*)buffer = regId;
	/* move the pointer forward by 2 bytes */
	buffer += sizeof(regId);

	uint32_t len = getSize(regId);

	return copy(value, buffer, len, sizeof(regTxBuffer) - (buffer - &regTxBuffer[2]));
}

uint32_t Register::copy(char* dest, char* source, uint32_t size, uint32_t freeSpace)
{
	/* return two so that we move by the reg ID and find a zero reg ID which terminates reception/transmission */
	if (freeSpace < size)
		return 0;

	memcpy(dest, source, size);
	return size + 2;
}

bool Register::prepareFrame(mab::Md80FrameId_E frameId, Md80Reg_E regId, char* value)
{
	/* if new frame */
	if (regTxPtr == nullptr)
	{ /* clear the buffer */
		memset(regTxBuffer, 0, sizeof(regTxBuffer));
		regTxBuffer[0] = frameId;
		regTxBuffer[1] = 0;
		regTxPtr = &regTxBuffer[2];
	}
	/* let pack know to fill data space with zeros */
	if (frameId == mab::Md80FrameId_E::FRAME_READ_REGISTER)
		value = nullptr;
	/* add value's data to tx buffer */
	uint32_t offset = pack(regId, value, regTxPtr);

	if (offset == 0)
	{
		throw "Error while packaging data. Make sure its size is not above 62 bytes. Remember to add 2 bytes per field (field ID).";
		return false;
	}

	regTxPtr += offset;
	return true;
}

uint16_t Register::getSize(uint16_t regId)
{
	if (regId == Md80Reg_E::commitHash)
		return 8;
	if (regId == Md80Reg_E::motorName)
		return 24;

	switch (getType(regId))
	{
		case type::I8:
		case type::U8:
			return 1;
		case type::I16:
		case type::U16:
			return 2;
		case type::I32:
		case type::U32:
		case type::F32:
			return 4;
		case type::STR:
		case type::UNKNOWN:
			return 0;
	}
	return 0;
}

Register::type Register::getType(uint16_t regId)
{
	switch (regId)
	{
		case Md80Reg_E::reverseDirection:
		case Md80Reg_E::motionModeStatus:
		case Md80Reg_E::motionModeCommand:
		case Md80Reg_E::homingMode:
		case Md80Reg_E::motorThermistorType:
		case Md80Reg_E::motorCalibrationMode:
		case Md80Reg_E::outputEncoderCalibrationMode:
		case Md80Reg_E::outputEncoderMode:
		case Md80Reg_E::bridgeType:
		case Md80Reg_E::outputEncoder:
		case Md80Reg_E::hardwareVersion:
		case Md80Reg_E::canTermination:
		case Md80Reg_E::motorShutdownTemp:
		case Md80Reg_E::runCalibrateCmd:
		case Md80Reg_E::runCalibrateOutpuEncoderCmd:
		case Md80Reg_E::runCalibratePiGains:
		case Md80Reg_E::runTestOutputEncoderCmd:
		case Md80Reg_E::runTestMainEncoderCmd:
		case Md80Reg_E::runSaveCmd:
		case Md80Reg_E::runHoming:
		case Md80Reg_E::runRestoreFactoryConfig:
		case Md80Reg_E::runReset:
		case Md80Reg_E::runClearWarnings:
		case Md80Reg_E::runClearErrors:
		case Md80Reg_E::runBlink:
		case Md80Reg_E::runZero:
		case Md80Reg_E::runCanReinit:
		case Md80Reg_E::brakeMode:
			return type::U8;
		case Md80Reg_E::motorTorgueBandwidth:
		case Md80Reg_E::canWatchdog:
		case Md80Reg_E::quickStatus:
		case Md80Reg_E::motorKV:
		case Md80Reg_E::state:
			return type::U16;
		case Md80Reg_E::outputEncoderDefaultBaud:
		case Md80Reg_E::canBaudrate:
		case Md80Reg_E::canId:
		case Md80Reg_E::motorPolePairs:
		case Md80Reg_E::mainEncoderErrors:
		case Md80Reg_E::outputEncoderErrors:
		case Md80Reg_E::calibrationErrors:
		case Md80Reg_E::bridgeErrors:
		case Md80Reg_E::hardwareErrors:
		case Md80Reg_E::communicationErrors:
		case Md80Reg_E::homingErrors:
		case Md80Reg_E::motionErrors:
		case Md80Reg_E::firmwareVersion:
		case Md80Reg_E::buildDate:
			return type::U32;
		case Md80Reg_E::targetPosition:
		case Md80Reg_E::targetVelocity:
		case Md80Reg_E::targetTorque:
		case Md80Reg_E::motorTorque:
		case Md80Reg_E::positionWindow:
		case Md80Reg_E::velocityWindow:
		case Md80Reg_E::maxTorque:
		case Md80Reg_E::maxVelocity:
		case Md80Reg_E::quickStopDeceleration:
		case Md80Reg_E::profileAcceleration:
		case Md80Reg_E::profileDeceleration:
		case Md80Reg_E::profileVelocity:
		case Md80Reg_E::homingMaxTravel:
		case Md80Reg_E::homingVelocity:
		case Md80Reg_E::homingTorque:
		case Md80Reg_E::homingPositionDeviationTrigger:
		case Md80Reg_E::positionLimitMax:
		case Md80Reg_E::positionLimitMin:
		case Md80Reg_E::shuntResistance:
		case Md80Reg_E::maxAcceleration:
		case Md80Reg_E::maxDeceleration:
		case Md80Reg_E::mainEncoderVelocity:
		case Md80Reg_E::mainEncoderPosition:
		case Md80Reg_E::mosfetTemperature:
		case Md80Reg_E::motorTemperature:
		case Md80Reg_E::motorInductance:
		case Md80Reg_E::motorResistance:
		case Md80Reg_E::motorImpPidKp:
		case Md80Reg_E::motorImpPidKd:
		case Md80Reg_E::motorPosPidKp:
		case Md80Reg_E::motorPosPidKi:
		case Md80Reg_E::motorPosPidKd:
		case Md80Reg_E::motorPosPidWindup:
		case Md80Reg_E::motorVelPidKp:
		case Md80Reg_E::motorVelPidKi:
		case Md80Reg_E::motorVelPidKd:
		case Md80Reg_E::motorVelPidWindup:
		case Md80Reg_E::motorFriction:
		case Md80Reg_E::motorStiction:
		case Md80Reg_E::outputEncoderDir:
		case Md80Reg_E::outputEncoderVelocity:
		case Md80Reg_E::outputEncoderPosition:
		case Md80Reg_E::motorGearRatio:
		case Md80Reg_E::calOutputEncoderStdDev:
		case Md80Reg_E::calOutputEncoderMinE:
		case Md80Reg_E::calOutputEncoderMaxE:
		case Md80Reg_E::calMainEncoderStdDev:
		case Md80Reg_E::calMainEncoderMinE:
		case Md80Reg_E::calMainEncoderMaxE:
		case Md80Reg_E::motorKt:
		case Md80Reg_E::motorKt_a:
		case Md80Reg_E::motorKt_b:
		case Md80Reg_E::motorKt_c:
		case Md80Reg_E::motorIMax:
			return type::F32;
		case Md80Reg_E::commitHash:
		case Md80Reg_E::motorName:
			return type::STR;
		default:
			return type::UNKNOWN;
	}
}

}  // namespace mab