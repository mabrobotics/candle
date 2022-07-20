#include "spiDevice.hpp"

static const char* spiDev = "/dev/spidev0.0";

// #define SPI_VERBOSE  1

SpiDevice::SpiDevice(char* rxBufferPtr, const int rxBufferSize_)
{
    fd = open(spiDev, O_RDWR);
    if(fd < 0) {
        printf("Could not open the SPI device...\r\n");
        exit(EXIT_FAILURE);
    }

    int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if(ret != 0) {
        printf("Could not read SPI mode...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if(ret != 0) {
        printf("Could not write SPI mode...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spiSpeed);
    if(ret != 0) {
        printf("Could not write the SPI max speed...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    rxBuffer = rxBufferPtr;
    rxBufferSize = rxBufferSize_;
    /* set transfer parameters that are constant in each cycle */
    trx.bits_per_word = 8;
    trx.speed_hz = spiSpeed;
}

SpiDevice::~SpiDevice()
{
    close(fd);
}


bool SpiDevice::transmit(char* buffer, int len, bool waitForResponse, int timeout, int responseLen)
{
    trx.tx_buf = (unsigned long)buffer;
    trx.rx_buf = (unsigned long)rxBuffer;
    trx.len = len;

    /* send */
    ioctl(fd, SPI_IOC_MESSAGE(1), &trx);

    if(waitForResponse)
    {
        if (receive(timeout, responseLen))
            return true;
        else
        {
            std::cout << "[SPI] Did not receive response from SPI device" << std::endl;
            return false;
        }
    }

    return true;
}

bool SpiDevice::receive(int timeout, int responseLen)
{

    memset(rxBuffer, 0, rxBufferSize);
    rxLock.lock();
    int delayUs = 10;
    int timeoutBusOutUs = timeout * 1000;
    uint8_t dummyTx = 0;
    uint8_t byteRx = 0;
    bool firstByteReceived = false;
    bytesReceived = 0;
    int usTimestamp = 0;
    const int timeoutUs = 100;

    while(usTimestamp < timeoutUs && timeoutBusOutUs > 0 && bytesReceived < responseLen)
    {
        trx.tx_buf = (unsigned long)&dummyTx;
        trx.rx_buf = (unsigned long)&byteRx;
        trx.len = 1;

        ioctl(fd, SPI_IOC_MESSAGE(1), &trx);
        timeoutBusOutUs -= delayUs; //If not receiving wait for 100ms and return false

        /* if the received byte is non-zero, continue with a larger transfer for the rest (responseLen -1) */
        if(byteRx != 0 )firstByteReceived = true;
        if(firstByteReceived)
        {   
            rxBuffer[bytesReceived++] = byteRx;
            uint8_t dummyTxa[maxResponseLen];
            memset(&dummyTxa, 0, responseLen);
            /* update SPI structure */
            trx.tx_buf = (unsigned long)&dummyTxa;
            trx.rx_buf = (unsigned long)&rxBuffer[1];
            trx.len = responseLen - 1;
            /* send request */
            ioctl(fd, SPI_IOC_MESSAGE(1), &trx);
            firstByteReceived += (responseLen - 1);
            break;
        }
        if(firstByteReceived && bytesReceived == 0)
            usTimestamp += delayUs; //If receiving wait for 100us idle state on the bus
        else 
            timeoutBusOutUs -= delayUs; //If not receiving wait for 100ms and return false
        usleep(delayUs);        
    }

    rxLock.unlock();

    #ifdef SPI_VERBOSE
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
        return true;

    return false;
}

bool SpiDevice::transmitReceive(char* buffer, int commandLen, int responseLen)
{
    memset(rxBuffer, 0, rxBufferSize);
    rxLock.lock();
    bytesReceived = 0;
    uint8_t txBuffer[maxResponseLen];
    memcpy(txBuffer,buffer,commandLen);

    trx.tx_buf = (unsigned long)txBuffer;
    trx.rx_buf = (unsigned long)rxBuffer;
    trx.len = responseLen > commandLen ? responseLen : commandLen;

    ioctl(fd, SPI_IOC_MESSAGE(1), &trx);
    bytesReceived = responseLen;
    rxLock.unlock();

    #ifdef SPI_VERBOSE
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
        return true;

    return false;
}