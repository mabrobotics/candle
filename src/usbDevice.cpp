#include "usbDevice.hpp"

#include <cstring>
#include <iostream>

#include "bus.hpp"
#include <libusb.h>

#define USB_VERBOSE 1

unsigned long hash(const char* str);

bool UsbDevice::init(u16 vid, u16 pid)
{
	struct libusb_device** devs;
	int rc = libusb_init(NULL);
	int32_t cnt = libusb_get_device_list(NULL, &devs);
	for (int i = 0; i < cnt; i++)
	{
		libusb_device* dev = devs[i];
		struct libusb_device_descriptor desc;
		rc = libusb_get_device_descriptor(dev, &desc);
		if (desc.idVendor == vid && desc.idProduct == pid)
		{
			rc = libusb_open(dev, &devh);
			u8 sNum[64];
			rc = libusb_get_string_descriptor_ascii(devh, desc.iSerialNumber, sNum, sizeof(sNum));
			serialDeviceId = atoll((char*)sNum);
			rc = libusb_get_string_descriptor_ascii(devh, desc.iProduct, sNum, sizeof(sNum));
			serialDeviceName = std::string((char*)sNum);
			for (int if_num = 0; if_num < 2; if_num++)
			{
				if (libusb_kernel_driver_active(devh, if_num))
					libusb_detach_kernel_driver(devh, if_num);

				rc = libusb_claim_interface(devh, if_num);
				if (rc < 0)
					return false;
			}
			break;
		}
	}
	libusb_free_device_list(devs, 1);
	return true;
}
bool UsbDevice::deinit()
{
	libusb_release_interface(devh, 0);
	libusb_close(devh);
	libusb_exit(nullptr);
	return true;
}

bool UsbDevice::transmit(char* buffer, int len, bool waitForResponse, int timeout, int responseLen,
						 bool faultVerbose)
{
	s32 sendLenActual = 0;
	s32 ret = libusb_bulk_transfer(devh, outEndpointAdr, (u8*)buffer, len, &sendLenActual, 10);
	if (ret < 0)
		return false;
	if (waitForResponse)
	{
		receive(responseLen, timeout);
	}
	return true;
}

bool UsbDevice::receive(int responseLen, int timeoutMs, bool checkCrc, bool faultVerbose)
{
	s32 ret =
		libusb_bulk_transfer(devh, inEndpointAdr, (u8*)rxBuffer, responseLen, &bytesReceived, timeoutMs);
	if (ret < 0)
		return false;
	return true;
}

unsigned long UsbDevice::getId() { return serialDeviceId; }
std::string UsbDevice::getDeviceName() { return serialDeviceName; }

UsbDevice::~UsbDevice() { deinit(); }

unsigned long hash(const char* str)
{
	unsigned long hash = 5381;
	int c;

	while ((c = *str++))
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

	return hash;
}
