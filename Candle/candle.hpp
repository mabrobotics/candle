#ifndef _CANDLE_H_
#define _CANDLE_H_

#include <usbDevice.hpp>

#include <string>
#include <thread>
#include <vector>
namespace mab
{
    enum CANdleMode_E
    {
        CONFIG,
        UPDATE
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
    enum CANdleBaudrate_E : uint8_t
    {
        CAN_BAUD_1M = 1,
        CAN_BAUD_5M = 5,
        CAN_BAUD_8M = 8
    };
    #pragma pack(push, 1)   //Ensures there in no padding (dummy) bytes in the structures below
    struct impedance_t
    {
        float kp;
        float kd;
        float torque_ff;
    };
    struct pidx_t
    {
        float kp, ki, kd, i_windup;
    };
    struct md80_t
    {
        uint16_t canId;
        uint8_t controlMode = (uint8_t)Md80Mode_E::IDLE;
        float positionTarget = 0.0f;
        float velocityTarget = 0.0f;
        float torqueSet = 0.0f;
        float maxTorque = 1.8f;
        float maxVelocity = 300.0f;
        pidx_t velocityController;
        pidx_t positionController;
        impedance_t impedanceController;
    };
    struct CanFrame_t
    {
        uint8_t length;
        uint8_t data[32];
    };

    struct StdMd80Frame_t
    {
        uint16_t canId;
        CanFrame_t toMd80;
    };

    struct stdUsbFrame_t
    {
        uint8_t id;
        std::vector<StdMd80Frame_t> md80Frames;
    };
    
    #pragma pack(pop)

    class Candle
    {
    private:
        UsbDevice*usb;
        std::thread receiverThread;
        std::thread transmitterThread;
        CANdleMode_E mode = CANdleMode_E::CONFIG;
        stdUsbFrame_t stdFrame;
        std::vector<md80_t> md80s;
        bool shouldStopReceiver;
        bool shouldStopTransmitter;

        void transmitNewStdFrame();

        void receive();
        void transmit();

        bool inUpdateMode();
        bool inConfigMode();
    public:
        Candle();
        ~Candle();
        bool transmitConfig(int canBaudrate, int canUpdateRateHz, int usbUpdateRateHz);
        
        bool addMd80(uint16_t canId);
        bool configMd80(uint16_t canId, float max_current, mab::Md80Mode_E mode);
        bool ping();
        bool configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout);

        
        void begin();
        void end();
        bool isOk();
    };
}
#endif