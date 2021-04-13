#ifndef _CANALIZATOR_H_
#define _CANALIZATOR_H_

#include <string>

class Candle
{
private:
    int fd;
    int canSpeed;
    int msgLen;
    int targetCanId;
    int timeout;
    unsigned char canRxBuffer[64];
    unsigned char canTxBuffer[64];
    unsigned char serialRxBuffer[128];
    unsigned char serialTxBuffer[128];
    int receivedBytes = 0;
public:
    Candle(std::string canalizatorDev, int baudrate, int canSpeed);
    Candle();
    ~Candle();
    int transmitAndReceive();
    bool transmit();
    int receive();

    bool setMsgLen(int newLen);
    bool setTargetId(int newId);
    bool setCanSpeed(int newBaudrate);
    bool setRxTimeout(int timeout);
    void setCanTx(const char* data, int len);
    int getCanRx(char * rxData);
    bool isOk();
};

#endif