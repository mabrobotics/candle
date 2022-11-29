
#include "register.hpp"

#include "candle.hpp"

namespace mab
{

bool Register::prepare(uint16_t canId, mab::Md80FrameId_E frameType)
{
	(void)frameType;
	/* clear the RX buffer and send register request */
	memset(regRxBuffer, 0, sizeof(regRxBuffer));
	regTxPtr = nullptr;
	regRxPtr = nullptr;
	return candle->sengGenericFDCanFrame(canId, sizeof(regTxBuffer), regTxBuffer, regRxBuffer, 100);
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
	switch (regId)
	{
		case Md80Reg_E::bridgeType:
		case Md80Reg_E::outputEncoder:
		case Md80Reg_E::hardwareVersion:
		case Md80Reg_E::canTermination:
		case Md80Reg_E::motorShutdownTemp:
			return 1;
		case Md80Reg_E::motorTorgueBandwidth:
		case Md80Reg_E::canWatchdog:
		case Md80Reg_E::errorVector:
		case Md80Reg_E::motorKV:
			return 2;
		case Md80Reg_E::mosfetTemperature:
		case Md80Reg_E::motorTemperature:
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
		case Md80Reg_E::outputEncoderVelocity:
		case Md80Reg_E::outputEncoderPosition:
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