#pragma once

#include <stdint.h>
#include <string.h>

#include <type_traits>

#include "mab_types.hpp"

namespace mab
{
/* forward declaration to deal with cyclic dependency*/
class Candle;

/* adding a new field:
1. add a new field in the Md80Reg_E enum (must be uniform with the same enum on MD80 side)
2. add it to the "switch case" in register.cpp in getType() this switch is also used for the size
3. add it to either RO/RW structs */

/* READ ONLY PARAMS */
typedef struct
{
	uint32_t firmwareVersion;
	uint8_t hardwareVersion;
	uint32_t buildDate;
	char commitHash[8];
	uint8_t bridgeType;
	float resistance;
	float inductance;
	uint16_t quickStatus;
	float mosfetTemperature;
	float motorTemperature;
	float mainEncoderVelocity;
	float mainEncoderPosition;
	float motorTorque;
	float outputEncoderVelocity;
	float outputEncoderPosition;
	float calOutputEncoderStdDev;
	float calOutputEncoderMinE;
	float calOutputEncoderMaxE;
	float calMainEncoderStdDev;
	float calMainEncoderMinE;
	float calMainEncoderMaxE;
	uint32_t mainEncoderErrors;
	uint32_t outputEncoderErrors;
	uint32_t calibrationErrors;
	uint32_t bridgeErrors;
	uint32_t hardwareErrors;
	uint32_t communicationErrors;
	uint32_t homingErrors;
	uint32_t motionErrors;
	float shuntResistance;
} regRO_st;

/* READ WRITE PARAMS */
typedef struct
{
	char motorName[24];
	uint32_t canId;
	uint32_t canBaudrate;
	uint16_t canWatchdog;
	uint8_t canTermination;
	uint32_t polePairs;
	uint16_t motorKV;
	uint8_t motorCalibrationMode;
	uint8_t motorThermistorType;
	float motorKt;
	float motorKt_a;
	float motorKt_b;
	float motorKt_c;
	float iMax;
	float gearRatio;
	uint8_t outputEncoder;
	uint8_t outputEncoderMode;
	uint8_t outputEncoderCalibrationMode;
	float outputEncoderDir;
	uint16_t torqueBandwidth;
	uint32_t outputEncoderDefaultBaud;
	float friction;
	float stiction;
	uint8_t motorShutdownTemp;
	ImpedanceControllerGains_t impedancePdGains;
	PidControllerGains_t velocityPidGains;
	PidControllerGains_t positionPidGains;
	uint8_t homingMode;
	float homingMaxTravel;
	float homingVelocity;
	float homingTorque;
	float positionLimitMax;
	float positionLimitMin;
	float maxAcceleration;
	float maxDeceleration;
	float maxTorque;
	float maxVelocity;
	float profileAcceleration;
	float profileDeceleration;
	float profileVelocity;
	float quickStopDeceleration;
	float positionWindow;
	float velocityWindow;
	float targetPosition;
	float targetVelocity;
	float targetTorque;
	uint8_t motionMode;
	uint16_t state;
	uint8_t reverseDirection;
	uint8_t brakeMode;
} regRW_st;

typedef struct
{
	regRO_st RO;
	regRW_st RW;
} regRead_st;

typedef struct
{
	regRW_st RW;
} regWrite_st;

typedef enum
{
	canId = 0x001,
	canBaudrate = 0x002,
	canWatchdog = 0x003,
	canTermination = 0x004,

	motorName = 0x010,
	motorPolePairs = 0x011,
	motorKt = 0x012,
	motorKt_a = 0x013,
	motorKt_b = 0x014,
	motorKt_c = 0x015,
	motorIMax = 0x016,
	motorGearRatio = 0x017,
	motorTorgueBandwidth = 0x018,
	motorFriction = 0x019,
	motorStiction = 0x01A,
	motorResistance = 0x01B,
	motorInductance = 0x01C,
	motorKV = 0x01D,
	motorCalibrationMode = 0x01E,
	motorThermistorType = 0x01F,

	outputEncoder = 0x020,
	outputEncoderDir = 0x021,
	outputEncoderDefaultBaud = 0x022,
	outputEncoderVelocity = 0x023,
	outputEncoderPosition = 0x024,
	outputEncoderMode = 0x025,
	outputEncoderCalibrationMode = 0x026,

	motorPosPidKp = 0x030,
	motorPosPidKi = 0x031,
	motorPosPidKd = 0x032,
	motorPosPidWindup = 0x034,

	motorVelPidKp = 0x040,
	motorVelPidKi = 0x041,
	motorVelPidKd = 0x042,
	motorVelPidWindup = 0x044,

	motorImpPidKp = 0x050,
	motorImpPidKd = 0x051,

	mainEncoderVelocity = 0x062,
	mainEncoderPosition = 0x063,
	motorTorque = 0x064,

	homingMode = 0x070,
	homingMaxTravel = 0x071,
	homingVelocity = 0x072,
	homingTorque = 0x073,
	homingPositionDeviationTrigger = 0x074,

	runSaveCmd = 0x080,
	runTestMainEncoderCmd = 0x081,
	runTestOutputEncoderCmd = 0x082,
	runCalibrateCmd = 0x083,
	runCalibrateOutpuEncoderCmd = 0x084,
	runCalibratePiGains = 0x085,
	runHoming = 0x086,
	runRestoreFactoryConfig = 0x087,
	runReset = 0x088,
	runClearWarnings = 0x089,
	runClearErrors = 0x08A,
	runBlink = 0x08B,
	runZero = 0x08C,
	runCanReinit = 0x08D,

	calOutputEncoderStdDev = 0x100,
	calOutputEncoderMinE = 0x101,
	calOutputEncoderMaxE = 0x102,
	calMainEncoderStdDev = 0x103,
	calMainEncoderMinE = 0x104,
	calMainEncoderMaxE = 0x105,

	positionLimitMax = 0x110,
	positionLimitMin = 0x111,
	maxTorque = 0x112,
	maxVelocity = 0x113,
	maxAcceleration = 0x114,
	maxDeceleration = 0x115,

	profileVelocity = 0x120,
	profileAcceleration = 0x121,
	profileDeceleration = 0x122,
	quickStopDeceleration = 0x123,
	positionWindow = 0x124,
	velocityWindow = 0x125,

	motionModeCommand = 0x140,
	motionModeStatus = 0x141,
	state = 0x142,

	targetPosition = 0x150,
	targetVelocity = 0x151,
	targetTorque = 0x152,

	brakeMode = 0x160,

	reverseDirection = 0x600,

	shuntResistance = 0x700,

	buildDate = 0x800,
	commitHash = 0x801,
	firmwareVersion = 0x802,
	hardwareVersion = 0x803,
	bridgeType = 0x804,
	quickStatus = 0x805,
	mosfetTemperature = 0x806,
	motorTemperature = 0x807,
	motorShutdownTemp = 0x808,
	mainEncoderErrors = 0x809,
	outputEncoderErrors = 0x80A,
	calibrationErrors = 0x80B,
	bridgeErrors = 0x80C,
	hardwareErrors = 0x80D,
	communicationErrors = 0x80E,
	homingErrors = 0x80F,
	motionErrors = 0x810,

} Md80Reg_E;
class Register
{
   public:
	enum class type
	{
		UNKNOWN = 0,
		U8 = 1,
		I8 = 2,
		U16 = 3,
		I16 = 4,
		U32 = 5,
		I32 = 6,
		F32 = 7,
		STR = 8
	};

	/**
	@brief Register object constructor
	@param candle Candle object pointer
	*/
	Register(Candle* candle) : candle(candle) {}
	/**
	@brief reads single-field registers
	@param canId ID of the drive
	@param regId first register's ID
	@param value first reference to a variable where the read value should be stored
	@param ...	remaining regId-value pairs to be read
	@return true if register was read
	*/
	template <typename T2, typename... Ts>
	bool read(uint16_t canId, Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		/* prepare and send the request frame */
		if (!prepare(canId, mab::Md80FrameId_E::FRAME_READ_REGISTER, regId, regValue, vs...))
			return false;
		/* interpret the frame */
		return interpret(canId, regId, regValue, vs...);
	}

	/**
	@brief writes single-field registers
	@param canId ID of the drive
	@param regId first register's ID
	@param value first reference to a value that should be written
	@param ...	remaining regId-value pairs to be written
	@return true if register was written
	*/
	template <typename T2, typename... Ts>
	bool write(uint16_t canId, Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		return prepare(canId, mab::Md80FrameId_E::FRAME_WRITE_REGISTER, regId, regValue, vs...);
	}

	/**
	@brief returns the size of a register based on it's id
	@param regId register's ID
	@return register size in bytes
	*/
	static uint16_t getSize(uint16_t regId);

	/**
	@brief returns the type of a register field based on it's id
	@param regId register's ID
	@return register type (Register::type enum class)
	*/
	static type getType(uint16_t regId);

   private:
	Candle* candle;

	static const uint32_t maxCanFramelen = 64;
	char regTxBuffer[maxCanFramelen];
	char regRxBuffer[maxCanFramelen];
	char* regTxPtr = nullptr;
	char* regRxPtr = nullptr;

	uint32_t pack(uint16_t regId, char* value, char* buffer);
	uint32_t unPack(uint16_t regId, char* value, char* buffer);
	uint32_t copy(char* dest, char* source, uint32_t size, uint32_t freeSpace);
	bool prepareFrame(mab::Md80FrameId_E frameId, Md80Reg_E regId, char* value);
	bool interpret(uint16_t canId);
	bool prepare(uint16_t canId, mab::Md80FrameId_E frameType);

	template <typename T2, typename... Ts>
	bool interpret(uint16_t canId, Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		/* if new frame */
		if (regRxPtr == nullptr)
			regRxPtr = &regRxBuffer[2];

		uint32_t offset = unPack(regId, (char*)&regValue, regRxPtr);
		if (offset == 0)
			return false;

		regRxPtr += offset;
		return interpret(canId, vs...);
	}

	template <typename T2, typename... Ts>
	bool prepare(uint16_t canId, mab::Md80FrameId_E frameType, Md80Reg_E regId, const T2& regValue, const Ts&... vs)
	{
		static_assert(!std::is_same<double, T2>::value, "register value should be float not double");
		if (!prepareFrame(frameType, regId, (char*)&regValue))
			return false;
		return prepare(canId, frameType, vs...);
	}
};

}  // namespace mab