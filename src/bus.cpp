#include "bus.hpp"

#include "candle_protocol.hpp"
namespace mab
{
Bus::Bus(mab::BusType_E type)
{
	busType = type;
}
Bus::~Bus()
{
	switch (busType)
	{
		case BusType_E::USB:
		{
			if (usb != NULL)
				delete usb;
			break;
		}
		case BusType_E::SPI:
		{
			if (spi != NULL)
				delete spi;
			break;
		}
		case BusType_E::UART:
		{
			if (uart != NULL)
				delete uart;
			break;
		}
	}
}

mab::BusType_E Bus::getType()
{
	return busType;
}

char* Bus::getRxBuffer(int index)
{
	return (char*)&rxBuffer[index];
}

bool Bus::transfer(char* buffer, int commandLen, bool waitForResponse, int timeout, int responseLen, bool faultVerbose)
{
	bool ret = false;

	switch (busType)
	{
		case BusType_E::USB:
		{
			ret = usb->transmit(buffer, commandLen, waitForResponse, timeout, faultVerbose);
			break;
		}
		case BusType_E::SPI:
		{
			if (buffer[0] == BUS_FRAME_UPDATE)
				ret = spi->transmitReceive(buffer, commandLen, responseLen);
			else
				ret = spi->transmit(buffer, commandLen, waitForResponse, timeout, responseLen, faultVerbose);
			break;
		}
		case BusType_E::UART:
		{
			ret = uart->transmit(buffer, commandLen, waitForResponse, timeout, faultVerbose);
			break;
		}
	}

	manageMsgCount(ret);
	return ret;
}

bool Bus::receive(int timeoutMs, bool checkCrc)
{
	bool ret = false;

	switch (busType)
	{
		case BusType_E::USB:
		{
			ret = usb->receive(timeoutMs);
			break;
		}
		case BusType_E::UART:
		{
			ret = uart->receive(timeoutMs, checkCrc);
			break;
		}
		default:
			break;
	}

	manageMsgCount(ret);

	return ret;
}

int Bus::getBytesReceived()
{
	switch (busType)
	{
		case BusType_E::USB:
		{
			return usb->getBytesReceived();
			break;
		}
		case BusType_E::SPI:
		{
			return spi->getBytesReceived();
			break;
		}
		case BusType_E::UART:
		{
			return uart->getBytesReceived();
			break;
		}
	}
	return 0;
}

void Bus::manageMsgCount(bool ret)
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
}
}  // namespace mab
