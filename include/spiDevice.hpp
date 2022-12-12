#pragma once

#include <fcntl.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>
#include <mutex>

#include "bus.hpp"
#include "crc.hpp"
#include "mab_types.hpp"

class SpiDevice : public mab::Bus
{
   public:
	SpiDevice(const std::string device = "/dev/spidev0.0");
	~SpiDevice();
	bool transmit(char* buffer, int len, bool waitForResponse = false, int timeout = 100, int responseLen = 0, bool faultVerbose = true) override;
	bool transfer(char* buffer, int commandLen, int responseLen) override;
	unsigned long getId() override;
	std::string getDeviceName() override;

	bool receive(int timeout, int responseLen, bool faultVerbose);

   private:
	Crc crc;

	/* SPI settings */
	std::string device;
	const uint8_t bits = 8;
	const uint32_t spiSpeed = 20000000;
	const uint8_t mode = SPI_MODE_0;
	static const uint32_t maxResponseLen = 2000;

	int fd;
	struct spi_ioc_transfer trx;
	std::mutex rxLock;

	void displayDebugMsg(char* buffer, int bytesReceived);
	void sendMessage(unsigned long request, spi_ioc_transfer* trx);
};
