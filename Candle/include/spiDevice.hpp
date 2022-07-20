#pragma once

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <iostream>
#include <mutex>
#include "mab_types.hpp"

class SpiDevice
{   

public:
    SpiDevice(char* rxBufferPtr, const int rxBufferSize_);
    ~SpiDevice();
    
    bool transmit(char* buffer, int len, bool waitForResponse = false, int timeout = 100, int responseLen = 0);
    bool receive(int timeout = 100, int responseLen = 0);
    bool transmitReceive(char* buffer, int commandLen, int responseLen);
    int getBytesReceived(){return bytesReceived;};

private:
    /* SPI settings */
    const uint8_t bits = 8;
    const uint32_t spiSpeed = 20000000;
    const uint8_t mode = SPI_MODE_0;
    static const uint32_t maxResponseLen = 2000;

    char* rxBuffer;
    int rxBufferSize;

    int fd;
    struct spi_ioc_transfer trx;
    int bytesReceived;
    std::mutex rxLock;
};
