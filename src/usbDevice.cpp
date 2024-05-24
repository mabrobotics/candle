#include "usbDevice.hpp"

#include <cstring>
#include <iostream>

#include "bus.hpp"
#include <libusb.h>

#define USB_VERBOSE 1

unsigned long hash(const char* str);

using std::cout, std::endl;

UsbDevice::UsbDevice(u16 vid, u16 pid, const std::vector<u32>& idsToIgnore, const std::string& id)
{
	busType						= mab::BusType_E::USB;
	struct libusb_device** devs = nullptr;

	int rc = libusb_init(NULL);
	if (rc < 0)
	{
		cout << "[USB] Failed to init libusb!" << endl;
		throw "Failed to init libusb!";
	}
	int32_t cnt = libusb_get_device_list(NULL, &devs);
	for (int i = 0; i < cnt; i++)
	{
		libusb_device*					dev = devs[i];
		struct libusb_device_descriptor desc;

		rc = libusb_get_device_descriptor(dev, &desc);
		if (desc.idVendor != vid || desc.idProduct != pid)
			continue;
		rc = libusb_open(dev, &devh);
		u8 sNum[64];
		rc = libusb_get_string_descriptor_ascii(devh, desc.iSerialNumber, sNum, sizeof(sNum));
		u32	 hashedId			   = hash((char*)sNum);
		u32	 hashedRequestedId	   = strtoul(id.c_str(), nullptr, 16);
		bool shouldContinue		   = false;
		bool shouldBreak		   = false;
		bool isSpecificIdRequested = (hashedRequestedId > 0);
		for (u32 ignoreId : idsToIgnore)
		{
			if (ignoreId == hashedRequestedId)
			{
				cout << "[USB] Device with requested ID: " << id
					 << " is already created! Quitting!" << endl;
				shouldBreak = true;
				throw("Device iwth ID " + id + " already created!");
				break;
			}
			if (ignoreId == hashedId)
			{
				shouldContinue = true;
				break;
			}
		}
		if (shouldContinue)
			continue;
		if (shouldBreak)
			break;
		if (isSpecificIdRequested && hashedRequestedId != hashedId)
			continue;

		serialDeviceId = hashedId;
		for (int if_num = 0; if_num < 2; if_num++)
		{
			if (libusb_kernel_driver_active(devh, if_num))
				libusb_detach_kernel_driver(devh, if_num);

			rc = libusb_claim_interface(devh, if_num);
			if (rc < 0)
			{
				cout << "[USB] Failed to claim interface!" << endl;
				throw "Failed to claim libusb interface!";
			}
		}
		break;
	}
	libusb_free_device_list(devs, 1);
	if (devh == nullptr)
	{
		cout << "[USB] CANdle not found on USB bus!" << endl;
		throw "Device not found in USB bus!";
	}
}
UsbDevice::~UsbDevice()
{
	libusb_release_interface(devh, 0);
	libusb_close(devh);
	libusb_exit(nullptr);
}

bool UsbDevice::transmit(
	char* buffer, int len, bool waitForResponse, int timeout, int responseLen, bool faultVerbose)
{
	(void)faultVerbose;
	s32 sendLenActual = 0;
	s32 ret = libusb_bulk_transfer(devh, outEndpointAdr, (u8*)buffer, len, &sendLenActual, 10);
	if (ret < 0)
	{
		cout << "[USB] Failed to transmit!" << endl;
		return false;
	}
	if (waitForResponse)
		receive(responseLen, timeout);
	return true;
}

bool UsbDevice::receive(int responseLen, int timeoutMs, bool checkCrc, bool faultVerbose)
{
	(void)checkCrc;
	(void)faultVerbose;
	s32 ret = libusb_bulk_transfer(
		devh, inEndpointAdr, (u8*)rxBuffer, responseLen, &bytesReceived, timeoutMs);
	if (ret < 0)
	{
		cout << "[USB] Failed to transmit!" << endl;
		return false;
	}
	return true;
}

unsigned long UsbDevice::getId() { return serialDeviceId; }

unsigned long hash(const char* str)
{
	unsigned long hash = 5381;
	int			  c;
	while ((c = *str++))
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	return hash;
}

// Search USB buses for multiple instances of the save (pid, vid) device.
// Returns number of devices found
u32 searchMultipleDevicesOnUSB(u16 pid, u16 vid)
{
	struct libusb_device** devs;

	u32 nDevices = 0;
	libusb_init(nullptr);
	u32 allDevices = libusb_get_device_list(nullptr, &devs);
	for (u32 i = 0; i < allDevices; i++)
	{
		struct libusb_device_descriptor desc;

		s32 ret = libusb_get_device_descriptor(devs[i], &desc);
		if (ret < 0)
			continue;
		if (desc.idProduct == pid && desc.idVendor == vid)
			nDevices++;
	}
	libusb_exit(nullptr);
	return nDevices;
}