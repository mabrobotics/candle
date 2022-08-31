#include "uartDevice.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include "crc.hpp"

//#define UART_VERBOSE
//#define UART_VERBOSE_ON_CRC_ERROR

static const char* uartDev = "/dev/ttyAMA0";

UartDevice::UartDevice(char* rxBufferPtr, const int rxBufferSize_)
{
	fd = open(uartDev, O_RDWR);

	if (tcgetattr(fd, &tty) != 0)
	{	
		const char* msg = "[UART] Could not open the UART device... (is UART bus available on your device?)";
		std::cout << msg << std::endl;
		throw msg;
	}

	tty.c_cflag &= ~PARENB;	 // Clear parity bit, disabling parity (most common)
	// tty.c_cflag |= PARODD;  // Set parity to ODD
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

	gotResponse = false;

	rxBuffer = rxBufferPtr;
	rxBufferSize = rxBufferSize_;

	crc = new Crc();

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
	delete (crc);
}

bool UartDevice::transmit(char* buffer, int commandLen, bool _waitForResponse, int timeout, bool faultVerbose)
{
	commandLen = crc->addCrcToBuf(buffer, commandLen);

	if (write(fd, buffer, commandLen) == -1)
	{
		std::cout << "[UART] Writing to UART Device failed. Device Unavailable!" << std::endl;
		return false;
	}
	if (_waitForResponse)
		return receive(timeout, true, faultVerbose);
	return true;
}

bool UartDevice::receive(int timeoutMs, bool checkCrc, bool faultVerbose)
{
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

	/* check CRC */
	if (crc->checkCrcBuf(rxBuffer, bytesReceived) && checkCrc)
		bytesReceived = bytesReceived - crc->getCrcLen();
	else if (bytesReceived > 0 && checkCrc)
	{
#ifdef UART_VERBOSE_ON_CRC_ERROR
		displayDebugMsg(rxBuffer, bytesReceived);
#endif

		errorCnt++;
		/* clear the command byte -> the frame will be rejected */
		rxBuffer[0] = 0;
		bytesReceived = 0;
		std::cout << "[UART] ERROR CRC!" << std::endl;
	}
	else if (bytesReceived == 0)
		if (faultVerbose) std::cout << "[UART] Did not receive response from UART Device." << std::endl;

#ifdef UART_VERBOSE
	displayDebugMsg(rxBuffer, bytesReceived);
#endif
	if (bytesReceived > 0)
	{
		gotResponse = true;
		return true;
	}
	return false;
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