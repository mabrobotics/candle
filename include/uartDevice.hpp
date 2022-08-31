#pragma once

#include <fcntl.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <mutex>

#include "crc.hpp"
#include "mab_types.hpp"

class UartDevice
{
   public:
	UartDevice(char* rxBufferPtr, const int rxBufferSize_);
	~UartDevice();
	bool transmit(char* buffer, int len, bool waitForConfirmation = false, int timeout = 100, bool faultVerbose = true);
	bool receive(int timeout = 100, bool checkCrc = true, bool faultVerbose = true);
	int getBytesReceived() { return bytesReceived; };
	uint32_t getErrorCnt() { return errorCnt; }

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
