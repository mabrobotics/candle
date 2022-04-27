#pragma once

#include <cstdint>
/**
 * 
 * @brief This file contains Definitions for communicating between host <-> CANdle. 
 * This should not be required by the user during the creation of custom code. * 
 */
namespace mab
{
    enum UsbFrameId_t : uint8_t
    {
        USB_FRAME_NONE = 0,
        USB_FRAME_PING_START = 1,
        USB_FARME_CANDLE_CONFIG_BAUDRATE = 2,
        USB_FRAME_MD80_ADD = 3,
        USB_FRAME_MD80_GENERIC_FRAME = 4,
        USB_FRAME_MD80_CONFIG_CAN = 5,
        USB_FRAME_BEGIN = 6,
        USB_FRAME_END = 7,
        USB_FRAME_UPDATE = 8,
        USB_FRAME_RESET = 9,
    };

#pragma pack(push, 1)   //Ensures there in no padding (dummy) bytes in the structures below
    struct GenericMd80Frame32
    {
        uint8_t frameId;
        uint8_t canMsgLen; 
        uint8_t timeoutMs = 1;
        uint16_t driveCanId;
        uint8_t canMsg[32];
    };
    struct GenericMd80Frame64
    {
        uint8_t frameId;
        uint8_t canMsgLen = 64; 
        uint8_t timeoutMs = 2;
        uint16_t driveCanId;
        uint8_t canMsg[64];
    };
    struct AddMd80Frame_t
    {
        uint8_t id;
        uint16_t driveAdress;
    };
    struct ConfigMd80Frame_t
    {
        uint8_t id;
        uint16_t driveAdress;
        uint8_t control_mode;
        float maxCurrent;
    };
#pragma pack(pop)
}
