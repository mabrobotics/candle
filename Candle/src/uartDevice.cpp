#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <stdbool.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include "uartDevice.hpp"

// #define UART_VERBOSE   1

static const char* uartDev = "/dev/ttyAMA0";

UartDevice::UartDevice(char* rxBufferPtr, const int rxBufferSize_)
{
    fd = open(uartDev,  O_RDWR);

    if(tcgetattr(fd, &tty) != 0) 
    {
        std::cout<<"Error "<< errno <<" from tcgetattr: " << strerror(errno)<<std::endl;
        return;
    }
    
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 0;   
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B2000000);
    cfsetospeed(&tty, B2000000);

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cout<<"Error "<< errno <<" from tcgetattr: " << strerror(errno)<<std::endl;
        return;
    }

    gotResponse = false;

    rxBuffer = rxBufferPtr;
    rxBufferSize = rxBufferSize_;

    /* frame used to automatically detect baudrate on Slave device side */
    char detectFrame = 0x55;
    transmit(&detectFrame,1);    
}
bool UartDevice::transmit(char* buffer, int len, bool _waitForResponse, int timeout)
{
    if (write(fd, buffer, len) == -1)
    {
        std::cout << "[UART] Writing to Uart Device failed. Device Unavailable!" << std::endl;
        return false;
    }
    if(_waitForResponse)
    {
        if (receive(timeout))
            return true;
        else
        {
            std::cout << "[UART] Did not receive response from Uart Device." << std::endl;
            return false;
        }
    }
    return true;
}
bool UartDevice::receive(int timeoutMs)
{    
    memset(rxBuffer, 0, rxBufferSize);
    rxLock.lock();
    const int delayUs = 10;
    const int timeoutUs = 100;
    int timeoutBusOutUs = timeoutMs * 1000;
    int usTimestamp = 0;
    bytesReceived = 0;
    bool firstByteReceived = false;
    while(usTimestamp < timeoutUs && timeoutBusOutUs > 0)
    { 
        char newByte;
        int bytesRead = read(fd, &newByte, 1);
        if(bytesRead > 0)
        {
            firstByteReceived = true;
            rxBuffer[bytesReceived++] = newByte;
            continue;
        }
        if(firstByteReceived && bytesRead == 0)
            usTimestamp += delayUs; //If receiving wait for 100us idle state on the bus
        else 
            timeoutBusOutUs -= delayUs; //If not receiving wait for 100ms and return false
        usleep(delayUs);        
    }
    rxLock.unlock();
#ifdef UART_VERBOSE
    if(bytesReceived > 0)
    {
        std::cout << "Got " << std::dec << bytesReceived  << "bytes." <<std::endl;
        std::cout << rxBuffer << std::endl;
        for(int i = 0; i < bytesReceived; i++)
            std::cout << std::hex << "0x" << (unsigned short)rxBuffer[i] << " ";
        std::cout << std::dec << std::endl << "#######################################################" << std::endl; 
    }
#endif
    if(bytesReceived > 0)
    {
        gotResponse = true;
        return true;
    }
    return false;
}
UartDevice::~UartDevice()
{
    close(fd);
}