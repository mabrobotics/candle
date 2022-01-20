namespace mab
{
    struct RegImpedance_t
    {
        float kp;
        float kd;
        float torque_ff;
    };
    struct RegPid_t
    {
        float kp, ki, kd, i_windup;
    };

    enum Md80Mode_E : uint8_t
    {
        IDLE,
        POSITION_PID,
        VELOCITY_PID,
        TORQUE,
        IMPEDANCE
    };
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
        FRAME_STOP_WDG			= 0x19,
        FRAME_CAN_CONFIG		= 0x20,
        FRAME_CAN_SAVE			= 0x21,
        FRAME_DIAGNOSTIC		= 0x69,
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
} // namespace mab
