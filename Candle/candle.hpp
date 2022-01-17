#ifndef _CANDLE_H_
#define _CANDLE_H_

#include <usbDevice.hpp>

#include <string>
#include <thread>
namespace mab
{
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
    public:
        Candle(std::string canalizatorDev);
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