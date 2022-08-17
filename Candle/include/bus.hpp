#pragma once

#include "mab_types.hpp"
#include "spiDevice.hpp"
#include "uartDevice.hpp"
#include "usbDevice.hpp"

namespace mab
{
enum class BusType_E
{
	USB,
	SPI,
	UART
};

class Bus
{
   public:
	UsbDevice* usb = NULL;
	SpiDevice* spi = NULL;
	UartDevice* uart = NULL;

	Bus(mab::BusType_E type);
	~Bus();
	mab::BusType_E getType();
	char* getRxBuffer(int index = 0);
	bool transfer(char* buffer, int len, bool waitForResponse = false, int timeout = 100, int responseLen = 0, bool faultVerbose = true);
	bool receive(int timeoutMs = 100, bool checkCrc = true);
	int getBytesReceived();
	int getRxBufferSize() { return rxBufferSize; };
	int getTxBufferSize() { return txBufferSize; };

   private:
	static const int errorThreshold = 5;
	static const int msgCntThreshold = 1000;
	int errorCnt = 0;
	int msgCnt = 0;

	BusType_E busType;
	static const int rxBufferSize = 1024;
	static const int txBufferSize = 1024;
	char rxBuffer[rxBufferSize];
	char txBuffer[txBufferSize];

	void manageMsgCount(bool ret);
};
}  // namespace mab
