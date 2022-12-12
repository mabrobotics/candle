#include "bus.hpp"

namespace mab
{

bool Bus::transfer(char* buffer, int commandLen, int responseLen)
{
	/* suppres unused variable warnings */
	[buffer, commandLen, responseLen] {};
	return false;
}
bool Bus::receive(int responseLen, int timeoutMs, bool checkCrc, bool faultVerbose)
{
	[responseLen, timeoutMs, checkCrc, faultVerbose] {};
	return false;
}
void Bus::flushReceiveBuffer()
{
	return;
}

/* public non-virtual functions */
int Bus::getBytesReceived()
{
	return bytesReceived;
}
mab::BusType_E Bus::getType()
{
	return busType;
}
char* Bus::getRxBuffer(int index)
{
	return (char*)&rxBuffer[index];
}

bool Bus::manageMsgErrors(bool ret)
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
}