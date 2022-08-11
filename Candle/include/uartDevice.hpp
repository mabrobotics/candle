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
#include "crc.hpp"

class UartDevice
{   

public:
    UartDevice(char* rxBufferPtr, const int rxBufferSize_);
    ~UartDevice();
    bool transmit(char* buffer, int len, bool waitForConfirmation = false, int timeout = 100, bool faultVerbose = true);
    bool receive(int timeout = 100, bool checkCrc = true, bool faultVerbose = true);
    int getBytesReceived(){return bytesReceived;};
    uint32_t getErrorCnt(){return errorCnt;}

private:
    Crc* crc;
    uint32_t errorCnt;

    const uint32_t uartSpeed = B2000000;

    char* rxBuffer;
    int rxBufferSize;

    int fd;
    struct termios tty;
    int bytesReceived;
    bool gotResponse;
    std::mutex rxLock;

    void displayDebugMsg(char* buffer, int bytesReceived);
};
