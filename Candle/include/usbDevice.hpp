#pragma once

#include <string>
#include <mutex>
#include <vector>

class UsbDevice
{
public:
    UsbDevice(std::string deviceName, std::string idVendor, std::string idProduct);
    ~UsbDevice();
    bool transmit(char* buffer, int len, bool waitForConfirmation = false, int timeout = 100);
    bool receive(int timeout = 100);
    unsigned long getId()   {return serialDeviceId;}

    static const int rxBufferSize = 1024;
    char rxBuffer[rxBufferSize];
    int bytesReceived;

    std::string getSerialDeviceName() {return serialDeviceName;}
    static std::vector<std::string> getConnectedACMDevices(std::string idVendor, std::string idProduct);
    static unsigned long getConnectedDeviceId(std::string devName);
private:
    int fd;
    std::string serialDeviceName;
    unsigned long serialDeviceId;
    int timeouttRx;
    bool waitingForResponse;
    bool gotResponse;
    std::mutex rxLock;
};
