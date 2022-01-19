
#include <cstdint>

enum UsbFrameId_t : uint8_t
{
    USB_FRAME_NONE,
    USB_FRAME_PING_START,
	USB_FRAME_MD80_ADD,
	USB_FRAME_MD80_CONFIG_DRIVE,
    USB_FRAME_MD80_CONFIG_CAN,
	USB_FRAME_BEGIN,
	USB_FRAME_END,
};

#pragma pack(push, 1)   //Ensures there in no padding (dummy) bytes in the structures below
struct ConfigFrame_t
{
    uint8_t id;
    uint32_t CanBaudrate;
    uint32_t CanUpdateRate;
    uint32_t UsbUpdateRate;
};
struct GenericMd80Frame
{
    uint8_t frameId;
    uint8_t canMsgLen; 
    uint16_t driveCanId;
    uint8_t canMsg[32];
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