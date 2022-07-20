#include "bus.hpp"
#include "candle_protocol.hpp"
namespace mab
{
    Bus::Bus(mab::BusType_E type)
    {
        busType = type;
    }
    Bus::~Bus()
    {
        switch(busType)
        {
            case BusType_E::USB:
            {
                delete usb;
                break;
            }
            case BusType_E::SPI:
            {
                delete spi;
                break;
            }
            case BusType_E::UART:
            {
                delete uart;
                break;
            }
        }
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
                if(buffer[0] == USB_FRAME_UPDATE)
                    return spi->transmitReceive(buffer,len,responseLen);
                else
                    return spi->transmit(buffer,len,waitForResponse,timeout,responseLen);
                break;
            }
            case BusType_E::UART:
            {
                return uart->transmit(buffer,len,waitForResponse,timeout);
                break;
            }
        }
        return false;
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
            case BusType_E::UART:
            {
                return uart->receive(timeoutMs);
                break;
            }
        }
        return false;
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
                return uart->getBytesReceived();
                break;
            }
        }
        return 0;
    }
}
