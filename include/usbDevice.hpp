#pragma once

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include "bus.hpp"

class UsbDevice : public mab::Bus
{
	static constexpr int inEndpointAdr	= 0x81;	 ///< CANdle USB input endpoint address.
	static constexpr int outEndpointAdr = 0x01;	 ///< CANdle USB output endpoint address.
  public:
	UsbDevice(u16					  vid		  = 0x0000,
			  u16					  pid		  = 0x0000,
			  const std::vector<u32>& idsToIgnore = {},
			  const std::string&	  requestID	  = "");
	~UsbDevice();
	bool transmit(char* buffer,
				  int	len,
				  bool	waitForResponse = false,
				  int	timeout			= 100,
				  int	responseLen		= 0,
				  bool	faultVerbose	= true) override;
	bool receive(int  responseLen,
				 int  timeoutMs	   = 100,
				 bool checkCrc	   = true,
				 bool faultVerbose = true) override;

	unsigned long getId() override;
	std::string	  getDeviceName() override { return ""; }

  private:
	unsigned long serialDeviceId = 0;

	struct libusb_device_handle* devh = nullptr;
};

u32 searchMultipleDevicesOnUSB(u16 pid, u16 vid);
