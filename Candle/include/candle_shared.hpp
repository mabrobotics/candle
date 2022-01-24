#pragma once
#include "mab_types.hpp"
#include "md80.hpp"

#include <vector>
namespace mab
{
    enum CANdleMode_E
    {
        CONFIG,
        UPDATE
    };

    enum CANdleBaudrate_E : uint8_t
    {
        CAN_BAUD_1M = 1,
        CAN_BAUD_5M = 5,
        CAN_BAUD_8M = 8
    };

    class Candle
    {
    public:
        Candle(CANdleBaudrate_E canBaudrate);
        ~Candle();
        std::vector<Md80> md80s;

        /**
        Sends a FDCAN Frame to IDs in range (10 - 2047), and checks for valid responses from Md80;
        @param a a scalar
        @param b a vector
        @return the vector with each component multiplied by the scalar
        */
        std::vector<uint16_t> ping();

        bool addMd80(uint16_t canId);
        bool configCandleBaudrate(CANdleBaudrate_E canBaudrate);

        bool configMd80Can(uint16_t canId, uint16_t newId, CANdleBaudrate_E newBaudrateMbps, unsigned int newTimeout);
        bool configMd80Save(uint16_t canId);
        bool configMd80SetZero(uint16_t canId);
        bool configMd80SetCurrentLimit(uint16_t canId, float currentLimit);
        
        bool setupMd80Calibration(uint16_t canId);

        bool controlMd80Enable(uint16_t canId, bool enable);
        bool controlMd80Mode(uint16_t canId, Md80Mode_E mode);

        Md80* getMd80FromList(uint16_t id);
        
        bool begin();
        bool end();
        bool reset();
    };
}
