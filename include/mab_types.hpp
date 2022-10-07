#pragma once

#include <cstdint>
namespace mab
{
/**
 * @brief Impedance regulator parameters
 *
 * Impedance regulator output is computed as: torque = kp * position_error + kd * velocity_error + torque_ff;
 */
struct RegImpedance_t
{
	float kp;
	float kd;
	float torque_ff;
};
/**
 * @brief PID regulator parameters. This is used to setup either Position PID regulator or Velocity PID regulator.
 * @note i_windup is an anti-windup parameter. This limits the maximum output of the integral (i) part of the regulator.
 */
struct RegPid_t
{
	float kp, ki, kd, i_windup;
};

/**
 * @brief Md80 Control Mode
 * @note Position PID is a cascade regulator, output of the Position PID (target velocity) is passed as an input
 *  of Velocity PID. Velocity PID output (torque) is then passed directly to internal current/torque controller.
 */
enum Md80Mode_E : uint8_t
{
	IDLE = 0,		  /*!< Idle mode, no control output */
	POSITION_PID = 1, /*!< Position PID mode (cascade regulators) */
	VELOCITY_PID = 2, /*!< Velocity PID mode */
	DEPRECATED = 3,	  /*!< This mode is deprecated and left for compatibility reasons. Do not use it */
	IMPEDANCE = 4,	  /*!< Impedance mode, uses Impedance controller similar to spring-damper system */
};
/**
 * @brief FDCAN frame ids supported by Md80
 */
enum Md80FrameId_E : uint8_t
{
	FRAME_FLASH_LED = 0x00,
	FRAME_MOTOR_ENABLE = 0x01,
	FRAME_CONTROL_SELECT = 0x02,
	FRAME_ZERO_ENCODER = 0x03,
	FRAME_BASE_CONFIG = 0x04,
	FRAME_GET_INFO = 0x05,
	FRAME_SET_BANDWIDTH = 0x06,
	FRAME_POS_CONTROL = 0x10,
	FRAME_VEL_CONTROL = 0x11,
	FRAME_IMP_CONTROL = 0x12,
	FRAME_RESTART = 0x13,
	FRAME_SET_MOTION_TARGETS = 0x14,
	FRAME_CAN_CONFIG = 0x20,
	FRAME_CAN_SAVE = 0x21,
	FRAME_WRITE_REGISTER = 0x40,
	FRAME_READ_REGISTER = 0x41,
	FRAME_DIAGNOSTIC = 0x69,
	FRAME_CALIBRATION = 0x70,
	FRAME_MOTOR_CONFIG = 0x71,
	FRAME_MOTOR_MOTION_CONFIG = 0x72,
	FRAME_DIAGNOSTIC_EXTENDED = 0x73,
	RESPONSE_DEFAULT = 0xA0
};
struct CanFrame_t
{
	uint8_t length;
	uint8_t data[32];
};
struct StdMd80CommandFrame_t
{
	uint16_t canId;
	CanFrame_t toMd80;
};
struct StdMd80ResponseFrame_t
{
	uint16_t canId;
	CanFrame_t fromMd80;
};

// ########################3

typedef struct
{
	float kp;
	float kd;
	float outMax;
} ImpedanceControllerGains_t;

typedef struct
{
	float kp;
	float ki;
	float kd;
	float intWindup;
	float outMax;
} PidControllerGains_t;

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
	uint16_t errorVector;
	float temperature;
	float temperatureAux;
} regRO_st;

/* READ WRITE PARAMS */
typedef struct
{
	char motorName[24];
	uint32_t canId;
	uint32_t canBaudrate;
	uint16_t canWatchdog;
	uint32_t polePairs;
	uint16_t motorKV;
	float motorKt;
	float motorKt_a;
	float motorKt_b;
	float motorKt_c;
	float iMax;
	float gearRatio;
	uint8_t outputEncoder;
	float outputEncoderDir;
	uint16_t torqueBandwidth;
	uint32_t outputEncoderDefaultBaud;
	float friction;
	float stiction;
	ImpedanceControllerGains_t impedancePdGains;
	PidControllerGains_t velocityPidGains;
	PidControllerGains_t positionPidGains;
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

/* adding a new field:
1. add a new filed below (must be uniform with the same enum on MD80 side)
2. add it to the "switch case" in md80.cpp (regarding it's size)
3. add it to either RO/RW structs above */

enum Md80Reg_E : uint16_t
{
	canId = 0x001,
	canBaudrate = 0x002,
	canWatchdog = 0x003,

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

	outputEncoder = 0x020,
	outputEncoderDir = 0x021,
	outputEncoderDefaultBaud = 0x022,

	motorPosPidKp = 0x030,
	motorPosPidKi = 0x031,
	motorPosPidKd = 0x032,
	motorPosPidOutMax = 0x033,
	motorPosPidWindup = 0x034,

	motorVelPidKp = 0x040,
	motorVelPidKi = 0x041,
	motorVelPidKd = 0x042,
	motorVelPidOutMax = 0x043,
	motorVelPidWindup = 0x044,

	motorImpPidKp = 0x050,
	motorImpPidKd = 0x051,
	motorImpPidOutMax = 0x052,

	buildDate = 0x800,
	commitHash = 0x801,
	firmwareVersion = 0x802,
	hardwareVersion = 0x803,
	bridgeType = 0x804,
	errorVector = 0x805,
	temperature = 0x806,
	temperatureAux = 0x807,
};

}  // namespace mab
