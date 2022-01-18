#ifndef _CANDLE_H_
#define _CANDLE_H_

#include <usbDevice.hpp>

#include <string>
#include <thread>
#include <vector>
namespace mab
{
    struct impedance_t
    {
        float kp;
        float kd;
        float torque_ff;
    };
    struct pidx_t
    {
        float kp, ki, kd, i_windup, output_max;
    };
    struct md80_t
    {
        int adress;
        uint8_t controlMode;
        pidx_t velocityController;
        pidx_t positionController;
        impedance_t impedanceController;
    };
    struct CanFrame_t
    {
        uint8_t length;
        uint8_t data[64];
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
    enum CANdle_mode
    {
        CONFIG,
        UPDATE
    };
    enum MD80_mode : uint8_t
    {
        IDLE,
        POSITION_PID,
        VELOCITY_PID,
        TORQUE,
        IMPEDANCE
    };
    class Candle
    {
    private:
        UsbDevice*usb;
        std::thread receiverThread;
        std::thread transmitterThread;
        CANdle_mode mode = CANdle_mode::CONFIG;
        bool shouldStopReceiver;
        bool shouldStopTransmitter;
        void transmitNewStdFrame();

        bool inUpdateMode();
        bool inConfigMode();

        stdUsbFrame_t stdFrame;
        std::vector<md80_t> md80s;
    public:
        Candle();
        ~Candle();
        bool transmitConfig(int canBaudrate, int canUpdateRateHz, int usbUpdateRateHz);
        
        bool addMd80(int canId);
        bool configMd80(int adress, float max_current, mab::MD80_mode mode);

        void receive();
        void begin();
        void end();
        bool isOk();
    };
}
#endif