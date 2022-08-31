#pragma once

#include <mutex>
#include <string>
#include <vector>

class UsbDevice
{
   public:
	UsbDevice(std::string deviceName, std::string idVendor, std::string idProduct, char* rxBufferPtr, const int rxBufferSize_);
	~UsbDevice();
	bool transmit(char* buffer, int len, bool waitForConfirmation = false, int timeout = 100, bool faultVerbose = true);
	bool receive(int timeout = 100, bool faultVerbose = true);
	unsigned long getId() { return serialDeviceId; }
	std::string getSerialDeviceName() { return serialDeviceName; }
	static std::vector<std::string> getConnectedACMDevices(std::string idVendor, std::string idProduct);
	static unsigned long getConnectedDeviceId(std::string devName);
	int getBytesReceived() { return bytesReceived; };

   private:
	int bytesReceived;
	int rxBufferSize;
	char* rxBuffer;
	int fd;
	std::string serialDeviceName;
	unsigned long serialDeviceId;
	int timeouttRx;
	bool gotResponse;
	std::mutex rxLock;
};
