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
	virtual bool transfer(char* buffer, int commandLen, int responseLen)
	{
		(void)buffer;
		(void)commandLen;
		(void)responseLen;
		return false;
	}
	virtual bool receive(int responseLen, int timeoutMs = 100, bool checkCrc = true, bool faultVerbose = true)
	{
		(void)responseLen;
		(void)timeoutMs;
		(void)checkCrc;
		(void)faultVerbose;
		return false;
	}

	virtual unsigned long getId() = 0;
	virtual std::string getDeviceName() { return ""; }
	virtual void flushReceiveBuffer(){};

	/* public non-virtual functions */
	int getBytesReceived() { return bytesReceived; }
	mab::BusType_E getType() { return busType; }
	char* getRxBuffer(int index = 0) { return (char*)&rxBuffer[index]; }

	bool manageMsgErrors(bool ret)
	{
		msgCnt++;
		if (ret == false) errorCnt++;
		if (errorCnt > errorThreshold)
		{
			const char* msg = "Fatal communication error!";
			std::cout << msg << std::endl;
			throw msg;
		}
		if (msgCnt > msgCntThreshold)
		{
			errorCnt = 0;
			msgCnt = 0;
		}
		return ret;
	}

   private:
	static const uint32_t errorThreshold = 5;
	static const uint32_t msgCntThreshold = 1000;
	uint32_t msgCnt = 0;
	uint32_t errorCnt = 0;

   protected:
	BusType_E busType;
	int bytesReceived = 0;
	static const uint32_t rxBufferSize = 1024;
	static const uint32_t txBufferSize = 1024;
	char rxBuffer[rxBufferSize];
	char txBuffer[txBufferSize];
};
}  // namespace mab
