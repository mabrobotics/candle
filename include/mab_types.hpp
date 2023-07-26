#pragma once

#include <cstdint>
namespace mab
{
/**
 * @brief Impedance controller parameters
 *
 * Impedance controller output is computed as: torque = kp * position_error + kd * velocity_error + torque_ff;
 */
struct RegImpedance_t
{
	float kp;
	float kd;
	float torque_ff;
};
/**
 * @brief PID controller parameters. This is used to setup either Position PID controller or Velocity PID controller.
 * @note i_windup is an anti-windup parameter. This limits the maximum output of the integral (i) part of the controller.
 */
struct RegPid_t
{
	float kp, ki, kd, i_windup;
};

/**
 * @brief Md80 Control Mode
 * @note Position PID is a cascade controller, output of the Position PID (target velocity) is passed as an input
 *  of Velocity PID. Velocity PID output (torque) is then passed directly to internal current/torque controller.
 */
enum Md80Mode_E : uint8_t
{
	IDLE = 0,			  /*!< Idle mode, no control output */
	POSITION_PID = 1,	  /*!< Position PID mode (cascade controllers) */
	VELOCITY_PID = 2,	  /*!< Velocity PID mode */
	RAW_TORQUE = 3,		  /*!< Raw torque mode */
	IMPEDANCE = 4,		  /*!< Impedance mode, uses Impedance controller similar to spring-damper system */
	POSITION_PROFILE = 7, /*!< Position PID with trapezoidal profile (constant acceleration) */
	VELOCITY_PROFILE = 8, /*!< Velocity PID with trapezoidal profile (constant acceleration) */
};
/**
 * @brief FDCAN frame ids supported by Md80
 */
enum Md80FrameId_E : uint8_t
{

	FRAME_GET_INFO = 0x05,
	FRAME_POS_CONTROL = 0x10,
	FRAME_VEL_CONTROL = 0x11,
	FRAME_IMP_CONTROL = 0x12,
	FRAME_SET_MOTION_TARGETS = 0x14,
	FRAME_WRITE_REGISTER = 0x40,
	FRAME_READ_REGISTER = 0x41,
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

typedef union version_ut
{
	struct
	{
		char tag;
		uint8_t revision;
		uint8_t minor;
		uint8_t major;
	} s;
	uint32_t i;
} version_ut;

}  // namespace mab
