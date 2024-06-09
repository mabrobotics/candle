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
#include <array>

#include "bus.hpp"

struct termios tty;
struct termios ti_prev;

#define USB_VERBOSE 0

int open_device(std::string devName, std::string idVendor, std::string idProduct);
bool checkDeviceAvailable(std::string devName, std::string idVendor, std::string idProduct);
std::string getDeviceShortId(std::string devName);
unsigned long hash(const char* str);

UsbDevice::UsbDevice(const std::string idVendor, const std::string idProduct, std::vector<unsigned long> instances)
{
	auto listOfDevices = UsbDevice::getConnectedACMDevices(idVendor, idProduct);

	busType = mab::BusType_E::USB;

	if (listOfDevices.size() == 0)
	{
		std::cout << "[USB] No devices found!" << std::endl;
		throw "[USB] No devices found!";
	}

	if (instances.size() == 0)
		serialDeviceName = listOfDevices[0];
	else
	{
		for (auto& entry : listOfDevices)
		{
			unsigned int newIdCount = 0;
			for (auto& instance : instances)
				if (UsbDevice::getConnectedDeviceId(entry) != instance)
					newIdCount++;

			/* only if all instances were different from the current one -> create new device */
			if (newIdCount == instances.size())
			{
				serialDeviceName = entry;
				goto loopdone;
			}
		}
		const char* msg = "[USB] Failed to create USB object";
		throw msg;
	}

loopdone:

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
	tty.c_cc[VTIME] = 0;														  // Wait for up to 0.1s (1 decisecond), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	tcsetattr(device_descriptor, TCSANOW, &tty);  // Set the new serial config

	this->fd = device_descriptor;

	std::string setSerialCommand = "setserial " + getDeviceName() + " low_latency";
	if (system(setSerialCommand.c_str()) != 0)
		std::cout << "Could not execute command '" << setSerialCommand << "'. Communication in low-speed mode." << std::endl;
}

bool UsbDevice::transmit(char* buffer, int len, bool waitForResponse, int timeout, int responseLen, bool faultVerbose)
{
	errno = 0;
	if (write(fd, buffer, len) == -1)
	{
		std::cout << "[USB] Writing to USB Device failed. Device Unavailable! Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
		return manageMsgErrors(false);
	}
	if (waitForResponse)
	{
		if (receive(responseLen, timeout, faultVerbose))
			return manageMsgErrors(true);
		else
		{
			if (faultVerbose) std::cout << "[USB] Did not receive response from USB Device." << std::endl;
			return manageMsgErrors(false);
		}
	}
	return manageMsgErrors(true);
}

bool UsbDevice::receive(int responseLen, int timeoutMs, bool checkCrc, bool faultVerbose)
{
	(void)faultVerbose;
	(void)checkCrc;

	memset(rxBuffer, 0, rxBufferSize);
	rxLock.lock();
	bytesReceived = 0;

	using namespace std::chrono;
	long long timestampStart = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
	long long timestampAct = timestampStart;

	while ((timestampAct - timestampStart) < (timeoutMs * 1000) && responseLen > bytesReceived)
	{
		char newByte;
		int bytesRead = read(fd, &newByte, 1);
		if (bytesRead > 0)
		{
			rxBuffer[bytesReceived++] = newByte;
			continue;
		}
		usleep(1);
		timestampAct = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
	}

	rxLock.unlock();
#if USB_VERBOSE == 1
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
		return true;

	return false;
}

unsigned long UsbDevice::getId()
{
	return serialDeviceId;
}
std::string UsbDevice::getDeviceName()
{
	return serialDeviceName;
}

void UsbDevice::flushReceiveBuffer()
{
	/* the delay is required to make flush work */
	usleep(1000);
	tcflush(fd, TCOFLUSH);
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
