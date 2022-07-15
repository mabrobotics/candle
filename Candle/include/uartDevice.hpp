#pragma once

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <iostream>
#include <mutex>
#include <termios.h>  
#include "mab_types.hpp"


class UartDevice
{   

public:
    
    
    UartDevice(char* rxBufferPtr, const int rxBufferSize_);
    ~UartDevice();
    bool transmit(char* buffer, int len, bool waitForConfirmation = false, int timeout = 100);
    bool receive(int timeout = 100);
    int getBytesReceived(){return bytesReceived;};

private:

    const uint32_t uartSpeed = 460800;

    char* rxBuffer;
    int rxBufferSize;

    int fd;
    struct termios tty;
    int bytesReceived;
    bool gotResponse;
    std::mutex rxLock;
};
