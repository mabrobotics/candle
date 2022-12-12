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
	UartDevice(const std::string device = "/dev/ttyAMA0");
	~UartDevice();
	bool transmit(char* buffer, int len, bool waitForResponse = false, int timeout = 100, int responseLen = 0, bool faultVerbose = true) override;
	bool receive(int responseLen, int timeoutMs = 100, bool checkCrc = true, bool faultVerbose = true) override;
	unsigned long getId() override;
	std::string getDeviceName() override;
	void flushReceiveBuffer() override;

   private:
	Crc crc;

	/* UART settings */
	std::string device;
	const uint32_t uartSpeed = B1000000;

	int fd;
	struct termios tty;
	std::mutex rxLock;

	void displayDebugMsg(char* buffer, int bytesReceived);
};
