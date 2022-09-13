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

#include "bus.hpp"
#include "crc.hpp"
#include "mab_types.hpp"

class UartDevice : public mab::Bus
{
   public:
	UartDevice();
	~UartDevice();
	bool transmit(char* buffer, int len, bool waitForResponse = false, int timeout = 100, int responseLen = 0, bool faultVerbose = true) override;
	bool transfer(char* buffer, int commandLen, int responseLen) override;
	bool receive(int timeoutMs = 100, bool checkCrc = true, bool faultVerbose = true) override;
	int getBytesReceived() override { return bytesReceived; }
	unsigned long getId() override { return 0; }

	uint32_t getErrorCnt() { return errorCnt; };

   private:
	Crc* crc;
	uint32_t errorCnt;

	const uint32_t uartSpeed = B2000000;

	int fd;
	struct termios tty;
	int bytesReceived;
	bool gotResponse;
	std::mutex rxLock;

	void displayDebugMsg(char* buffer, int bytesReceived);
};
