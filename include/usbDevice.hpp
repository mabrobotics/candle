#pragma once

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include "bus.hpp"

class UsbDevice : public mab::Bus
{
   public:
	UsbDevice(){};
	UsbDevice(const std::string idVendor, const std::string idProduct, std::vector<unsigned long> instances);
	~UsbDevice();
	bool transmit(char* buffer, int len, bool waitForResponse = false, int timeout = 100, int responseLen = 0, bool faultVerbose = true) override;
	bool receive(int responseLen, int timeoutMs = 100, bool checkCrc = true, bool faultVerbose = true) override;
	unsigned long getId() override;
	std::string getDeviceName() override;
	void flushReceiveBuffer() override;

	static std::vector<std::string> getConnectedACMDevices(std::string idVendor, std::string idProduct);
	static unsigned long getConnectedDeviceId(std::string devName);

   private:
	int fd;
	std::string serialDeviceName;
	unsigned long serialDeviceId = 0;
	std::mutex rxLock;
};
