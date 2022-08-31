#include "usbDevice.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

struct termios tty;
struct termios ti_prev;

// #define USB_VERBOSE

int open_device(std::string devName, std::string idVendor, std::string idProduct);
bool checkDeviceAvailable(std::string devName, std::string idVendor, std::string idProduct);
std::string getDeviceShortId(std::string devName);
unsigned long hash(const char* str);

UsbDevice::UsbDevice(std::string deviceName, std::string idVendor, std::string idProduct, char* rxBufferPtr, const int rxBufferSize_)
{
	serialDeviceName = deviceName;
	int device_descriptor = open_device(serialDeviceName, idVendor, idProduct);
	serialDeviceId = getConnectedDeviceId(serialDeviceName);

	if (device_descriptor < 0)
	{
		const char* msg = "[USB] Device not found! Try re-plugging the device!";
		std::cout << msg << std::endl;
		throw msg;
		
	}

	tcgetattr(device_descriptor, &ti_prev);										  // Save the previous serial config
	tcgetattr(device_descriptor, &tty);											  // Read the previous serial config
	tty.c_cflag &= ~PARENB;														  // No parity
	tty.c_cflag &= ~CSTOPB;														  // One stop bit
	tty.c_cflag &= ~CSIZE;														  // Clear bit size setting
	tty.c_cflag |= CS8;															  // 8 Bits mode
	tty.c_cflag |= CREAD | CLOCAL;												  // Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty.c_lflag &= ~ICANON;														  // Disable canonical mode
	tty.c_lflag &= ~ECHO;														  // Disable echo
	tty.c_lflag &= ~ECHOE;														  // Disable erasure
	tty.c_lflag &= ~ECHONL;														  // Disable new-line echo
	tty.c_lflag &= ~ISIG;														  // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);										  // Disable software flow control
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);  // Disable special chars on RX
	tty.c_oflag &= ~OPOST;														  // Prevent special interpretation of output bytes
	tty.c_oflag &= ~ONLCR;														  // Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 0;  // Wait for up to 0.1s (1 decisecond), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// cfmakeraw(&tty);
	tcsetattr(device_descriptor, TCSANOW, &tty);  // Set the new serial config

	this->fd = device_descriptor;
	gotResponse = false;

	rxBuffer = rxBufferPtr;
	rxBufferSize = rxBufferSize_;
}

bool UsbDevice::transmit(char* buffer, int len, bool _waitForResponse, int timeout, bool faultVerbose)
{
	if (write(fd, buffer, len) == -1)
	{
		std::cout << "[USB] Writing to USB Device failed. Device Unavailable!" << std::endl;
		return false;
	}
	if (_waitForResponse)
	{
		if (receive(timeout, faultVerbose))
			return true;
		else
		{
			if (faultVerbose) std::cout << "[USB] Did not receive response from USB Device." << std::endl;
			return false;
		}
	}
	return true;
}

bool UsbDevice::receive(int timeoutMs, bool faultVerbose)
{
	(void)faultVerbose;
	memset(rxBuffer, 0, rxBufferSize);
	rxLock.lock();
	const int delayUs = 10;
	const int timeoutUs = 100;
	int timeoutBusOutUs = timeoutMs * 1000;
	int usTimestamp = 0;
	bytesReceived = 0;
	bool firstByteReceived = false;
	while (usTimestamp < timeoutUs && timeoutBusOutUs > 0)
	{
		char newByte;
		int bytesRead = read(fd, &newByte, 1);
		if (bytesRead > 0)
		{
			firstByteReceived = true;
			rxBuffer[bytesReceived++] = newByte;
			continue;
		}
		if (firstByteReceived && bytesRead == 0)
			usTimestamp += delayUs;	 // If receiving wait for 100us idle state on the bus
		else
			timeoutBusOutUs -= delayUs;	 // If not receiving wait for 100ms and return false
		usleep(delayUs);
	}
	rxLock.unlock();
#ifdef USB_VERBOSE
	if (bytesReceived > 0)
	{
		std::cout << "Got " << std::dec << bytesReceived << "bytes." << std::endl;
		std::cout << rxBuffer << std::endl;
		for (int i = 0; i < bytesReceived; i++)
			std::cout << std::hex << "0x" << (unsigned short)rxBuffer[i] << " ";
		std::cout << std::dec << std::endl
				  << "#######################################################" << std::endl;
	}
#endif
	if (bytesReceived > 0)
	{
		gotResponse = true;
		return true;
	}
	return false;
}

UsbDevice::~UsbDevice()
{
	ti_prev.c_cflag &= ~HUPCL;		   // This to release the RTS after close
	tcsetattr(fd, TCSANOW, &ti_prev);  // Restore the previous serial config
	close(fd);
}

std::vector<std::string> UsbDevice::getConnectedACMDevices(std::string idVendor, std::string idProduct)
{
	std::vector<std::string> deviceList;
	for (int i = 0; i < 10; i++)
	{
		std::string devName = "/dev/ttyACM" + std::to_string(i);
		if (checkDeviceAvailable(devName, idVendor, idProduct))
			deviceList.emplace_back(devName);
	}
	return deviceList;
}

unsigned long UsbDevice::getConnectedDeviceId(std::string devName)
{
	std::string shortId = getDeviceShortId(devName);
	return hash(shortId.c_str());
}

#include <sys/stat.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

std::string exec(const char* cmd)
{
	std::array<char, 128> buffer;
	std::string result;
	std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
	if (!pipe)
		throw std::runtime_error("popen() failed!");
	while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
		result += buffer.data();
	return result;
}

unsigned long hash(const char* str)
{
	unsigned long hash = 5381;
	int c;

	while ((c = *str++))
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

	return hash;
}

std::string getDeviceShortId(std::string devName)
{
	std::string cmdOutput = exec(std::string("udevadm info " + devName).c_str());
	std::stringstream result(cmdOutput);
	std::string line;
	while (getline(result, line))
	{
		if (line.find("ID_SERIAL_SHORT") != std::string::npos)
		{
			std::string idShort;
			std::stringstream lineStream(line);
			std::string word;
			std::vector<std::string> wordList;
			while (std::getline(lineStream, word, '='))
				wordList.push_back(word);
			idShort = wordList[wordList.size() - 1];
			return idShort;
		}
	}
	return "";
}

bool checkDeviceAvailable(std::string devName, std::string idVendor, std::string idProduct)
{
	std::string cmd = std::string("udevadm info ") + devName + std::string(" 2>/dev/null");
	std::string cmdOutput = exec(cmd.c_str());

	if (cmdOutput.size() < 5)
		return false;  // no device of name /dev/ttyACMx

	std::stringstream result(cmdOutput);
	std::string line;
	bool vendorOk = false, productOk = false;
	while (getline(result, line))
	{
		if (line.find(idVendor) != std::string::npos)
			vendorOk = true;
		if (line.find(idProduct) != std::string::npos)
			productOk = true;
	}
	if (!vendorOk || !productOk)
		return false;

	return true;
}

int open_device(std::string devName, std::string idVendor, std::string idProduct)
{
	if (checkDeviceAvailable(devName, idVendor, idProduct))
		return open(devName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	return -1;
}
