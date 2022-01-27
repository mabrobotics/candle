#include "usbDevice.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>  
#include <time.h>

#include <stdbool.h>
#include <stdio.h>

#include <cstring>
#include <iostream>


struct termios tty;
struct termios ti_prev;
pthread_mutex_t devLock;

// #define USB_VERBOSE

std::string open_device(int*fd);

UsbDevice::UsbDevice()
{
    int device_descriptor;

    serialDeviceName = open_device(&device_descriptor);
    if (device_descriptor < 0) 
    {
        std::cout << "CANdle device not found! Try re-plugging the dongle" << std::endl;
        exit(-1);
    }
    
    tcgetattr(device_descriptor, &ti_prev);    // Save the previous serial config
    tcgetattr(device_descriptor, &tty);         // Read the previous serial config
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;  // Clear bit size setting
    tty.c_cflag |= CS8;     // 8 Bits mode
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;     // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special chars on RX
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 0;    // Wait for up to 0.1s (1 decisecond), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    //cfmakeraw(&tty);
    tcsetattr(device_descriptor, TCSANOW, &tty);  // Set the new serial config 
    
    this->fd = device_descriptor;
    gotResponse = false;
    waitingForResponse = false;
}
bool UsbDevice::transmit(char* buffer, int len, bool _waitForResponse, int timeout)
{
    write(fd, buffer, len);
    if(_waitForResponse)
        if (receive(timeout))
            return true;
        else
        {
            std::cout << "Did not receive response from CANdle." << std::endl;
            return false;
        }
    return true;
}
bool UsbDevice::receive(int timeoutMs)
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
#ifdef USB_VERBOSE
    if(bytesReceived > 0)
    {
        std::cout << "Got " << std::dec << bytesReceived  << "bytes." <<std::endl;
        std::cout << rxBuffer << std::endl;
        for(int i = 0; i < bytesReceived; i++)
            std::cout << std::hex << "0x" << (int)rxBuffer[i] << " ";
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
UsbDevice::~UsbDevice()
{
    ti_prev.c_cflag &= ~HUPCL;        // This to release the RTS after close
    tcsetattr(fd, TCSANOW, &ti_prev); // Restore the previous serial config
    close(fd);
}


#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>

bool fileExists(std::string&filename)
{
    struct stat buffer;   
    if(!stat(filename.c_str() , &buffer) == 0)
        return false;
    return true;
}

std::string open_device(int *fd)
{       
    std::string baseDeviceName = "/dev/ttyACM";
    for(int i = 0; i < 10; i++)
    {
        //loop all ttyACMx devices
        std::string devName = baseDeviceName + std::to_string(i);
        if(fileExists(devName))
        {
            //if the ttyACMx exists check if it has CANdle descriptor in modalias file
            std::string modaliasFilePath = "/sys/class/tty/ttyACM" + std::to_string(i) + std::string("/device/modalias");
            std::ifstream modalias(modaliasFilePath);
            if(modalias.is_open())
            {
                //if modalias exists check if its contents mach CANdle descriptor (usb:vidpid)
                char modline[14];
                modalias.read(modline, 14);
                std::string modlineString(modline);
                std::string usbDevString("usb:v0069p1000");
                if(modlineString == usbDevString)
                {
                    *fd = open(devName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
                    return devName;
                }
            }
        }
    }
    *fd = -1;
    return "";
}
