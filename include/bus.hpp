#pragma once

#include "iostream"
#include "mab_types.hpp"
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
	virtual ~Bus() = default;
	virtual bool transmit(char* buffer, int len, bool waitForResponse = false, int timeout = 100, int responseLen = 0, bool faultVerbose = true) = 0;
	virtual bool transfer(char* buffer, int commandLen, int responseLen) = 0;
	virtual bool receive(int timeoutMs = 100, bool checkCrc = true, bool faultVerbose = true) = 0;
	virtual int getBytesReceived() = 0;
	virtual unsigned long getId() = 0;

	mab::BusType_E getType() { return busType; };
	char* getRxBuffer(int index = 0) { return (char*)&rxBuffer[index]; };
	int getRxBufferSize() { return rxBufferSize; };
	int getTxBufferSize() { return txBufferSize; };

   private:
	static const int errorThreshold = 5;
	static const int msgCntThreshold = 1000;
	int errorCnt = 0;
	int msgCnt = 0;

	BusType_E busType;

   protected:
	static const uint32_t rxBufferSize = 1024;
	static const uint32_t txBufferSize = 1024;
	char rxBuffer[rxBufferSize];
	char txBuffer[txBufferSize];
};
}  // namespace mab
