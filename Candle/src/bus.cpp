#include "bus.hpp"

namespace mab
{
    Bus::Bus(mab::BusType_E type)
    {
        busType = type;
    }
    Bus::~Bus()
    {

    }

    mab::BusType_E Bus::getType()
    {
        return busType;
    }

    char* Bus::getRxBuffer(int index)
    {
        return (char*)&rxBuffer[index];
    }   
    

    bool Bus::transfer(char* buffer, int len, bool waitForResponse, int timeout, int responseLen)
    {
        switch(busType)
        {
            case BusType_E::USB:
            {
                return usb->transmit(buffer,len,waitForResponse,timeout);
                break;
            }
            case BusType_E::SPI:
            {
                return spi->transmit(buffer,len,waitForResponse,timeout,responseLen);
                break;
            }
            case BusType_E::UART:
            {
                break;
            }
        }
    }

    bool Bus::receive(int timeoutMs, int responseLen)
    {
        switch(busType)
        {
            case BusType_E::USB:
            {
                return usb->receive(timeoutMs);
                break;
            }
            case BusType_E::SPI:
            {
                return spi->receiveUpdate(timeoutMs, responseLen);
                break;
            }
            case BusType_E::UART:
            {
                break;
            }
        }
    }

    int Bus::getBytesReceived()
    {
        
        switch(busType)
        {
            case BusType_E::USB:
            {
                return usb->getBytesReceived();
                break;
            }
            case BusType_E::SPI:
            {
                return spi->getBytesReceived();
                break;
            }
            case BusType_E::UART:
            {
                break;
            }
        }
    }
}
