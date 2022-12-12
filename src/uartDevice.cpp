#include "uartDevice.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include "bus.hpp"
#include "crc.hpp"

#define UART_VERBOSE			  0
#define UART_VERBOSE_ON_CRC_ERROR 0

UartDevice::UartDevice(const std::string device) : device(device)
{
	busType = mab::BusType_E::UART;

	fd = open(device.c_str(), O_RDWR);

	if (tcgetattr(fd, &tty) != 0)
	{
		const char* msg = "[UART] Could not open the UART device... (is UART bus available on your device?)";
		std::cout << msg << std::endl;
		throw msg;
	}

	tty.c_cflag &= ~PARENB;			// Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB;			// Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE;			// Clear all bits that set the data size
	tty.c_cflag |= CS8;				// 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS;		// Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL;	// Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;														  // Disable echo
	tty.c_lflag &= ~ECHOE;														  // Disable erasure
	tty.c_lflag &= ~ECHONL;														  // Disable new-line echo
	tty.c_lflag &= ~ISIG;														  // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);										  // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);  // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST;	// Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR;	// Prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	cfsetispeed(&tty, uartSpeed);
	cfsetospeed(&tty, uartSpeed);

	// Save tty settings, also checking for error
	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		std::cout << "[UART] Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
		return;
	}

	/* frame used to automatically detect baudrate on Slave device side -> send twice so that it can be easily discarded on the Candle slave device */
	char detectFrame[10] = {0x55, 0x55};
	if (write(fd, detectFrame, 2) == -1)
		std::cout << "[UART] Writing to UART Device failed. Device Unavailable!" << std::endl;
	/* allow the frame to be detected by the slave, before a next one is sent (the receiver timeout is set to a rather high value to ensure stable communication)*/
	usleep(20000);
}

UartDevice::~UartDevice()
{
	close(fd);
}

bool UartDevice::transmit(char* buffer, int len, bool waitForResponse, int timeout, int responseLen, bool faultVerbose)
{
	len = crc.addCrcToBuf(buffer, len);

	if (write(fd, buffer, len) == -1)
	{
		std::cout << "[UART] Writing to UART Device failed. Device Unavailable!" << std::endl;
		return manageMsgErrors(false);
	}
	if (waitForResponse)
		return manageMsgErrors(receive(responseLen, timeout, true, faultVerbose));
	return manageMsgErrors(true);
}

bool UartDevice::receive(int responseLen, int timeoutMs, bool checkCrc, bool faultVerbose)
{
	memset(rxBuffer, 0, rxBufferSize);
	rxLock.lock();
	bytesReceived = 0;

	using namespace std::chrono;
	long long timestampStart = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
	long long timestampAct = timestampStart;

	while ((timestampAct - timestampStart) < (timeoutMs * 1000) && bytesReceived < (int)(responseLen + crc.getCrcLen()))
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

	/* check CRC */
	if (crc.checkCrcBuf(rxBuffer, bytesReceived) && checkCrc)
		bytesReceived = bytesReceived - crc.getCrcLen();
	else if (bytesReceived > 0 && checkCrc)
	{
#if UART_VERBOSE_ON_CRC_ERROR == 1
		displayDebugMsg(rxBuffer, bytesReceived);
#endif
		/* clear the command byte -> the frame will be rejected */
		rxBuffer[0] = 0;
		bytesReceived = 0;
		std::cout << "[UART] ERROR CRC!" << std::endl;
	}
	else if (bytesReceived == 0)
		if (faultVerbose) std::cout << "[UART] Did not receive response from UART Device." << std::endl;

#if UART_VERBOSE == 1
	displayDebugMsg(rxBuffer, bytesReceived);
#endif
	if (bytesReceived > 0)
		return true;
	return false;
}

unsigned long UartDevice::getId()
{
	return 0;
}

std::string UartDevice::getDeviceName()
{
	return device;
}

void UartDevice::flushReceiveBuffer()
{
	/* the delay is required to make flush work */
	usleep(1000);
	tcflush(fd, TCOFLUSH);
}

void UartDevice::displayDebugMsg(char* buffer, int bytesReceived)
{
	if (bytesReceived > 0)
	{
		std::cout << "Got " << std::dec << bytesReceived << "bytes." << std::endl;
		std::cout << buffer << std::endl;
		for (int i = 0; i < bytesReceived; i++)
			std::cout << std::hex << "0x" << (unsigned short)buffer[i] << " ";
		std::cout << std::dec << std::endl
				  << "#######################################################" << std::endl;
	}
}