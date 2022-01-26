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
     * @note TORQUE mode directly sends torque to internal current/torque controller.
     */
    enum Md80Mode_E : uint8_t
    {
        IDLE = 0,           /*!< Idle mode, no control output */
        POSITION_PID = 1,   /*!< Position PID mode (cascade regulators) */
        VELOCITY_PID = 2,   /*!< Velocity PID mode */
        TORQUE = 3,         /*!< Torque mode, raw torque control, no regulators used */
        IMPEDANCE = 4,      /*!< Impedance mode, uses Impedance controller similar to spring-damper system */
    };
    /**
     * @brief FDCAN frame ids supported by Md80
     */
    enum Md80FrameId_E : uint8_t 
    {
        FRAME_FLASH_LED			= 0x00,
        FRAME_MOTOR_ENABLE 		= 0x01,
        FRAME_CONTROL_SELECT 	= 0x02,
        FRAME_ZERO_ENCODER		= 0x03,
        FRAME_BASE_CONFIG		= 0x04,
        FRAME_GET_INFO			= 0x05,
        FRAME_POS_CONTROL		= 0x10,
        FRAME_VEL_CONTROL		= 0x11,
        FRAME_IMP_CONTROL		= 0x12,
        FRAME_RESTART			= 0x13,
        FRAME_SET_MOTION_TARGETS= 0x14,
        FRAME_CAN_CONFIG		= 0x20,
        FRAME_CAN_SAVE			= 0x21,
        FRAME_DIAGNOSTIC        = 0x69,
        FRAME_CALIBRATION		= 0x70,
        RESPONSE_DEFAULT		= 0xA0
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
} // namespace mab
