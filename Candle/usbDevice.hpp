#pragma once

#include <string>
#include <mutex>

class UsbDevice
{
public:
    UsbDevice();
    ~UsbDevice();
    bool transmit(char* buffer, int len, bool waitForConfirmation = false, int timeout = 100);
    bool receive(int timeout = 100);

    static const int rxBufferSize = 512;
    char rxBuffer[rxBufferSize];
    int bytesReceived;

    std::string getSerialDeviceName() {return serialDeviceName;}

private:
    int fd;
    std::string serialDeviceName;
    int timeouttRx;
    bool waitingForResponse;
    bool gotResponse;
    std::mutex rxLock;
};
