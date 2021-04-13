#include "candle.hpp"

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include "unistd.h"
#define CAN_DEFAULT_SPEED 1000000

Candle::Candle()
{

}

Candle::Candle(std::string canalizatorDev, int baudrate = 115200, int canBaudrate = 1000000)
{
    fd = uart_init(&canalizatorDev[0], baudrate);
    if (fd < 0)
    {
        std:: cout << "Failed to open port " << canalizatorDev << std::endl;
        return;
    }
    setRxTimeout(5);
    setCanSpeed(canBaudrate);
    setMsgLen(0);
    memset(canRxBuffer, 0, sizeof(canRxBuffer));
    memset(canTxBuffer, 0, sizeof(canTxBuffer));
    memset(serialRxBuffer, 0, sizeof(serialRxBuffer));
    memset(serialTxBuffer, 0, sizeof(serialTxBuffer));
}

Candle::~Candle()
{
    uart_restore(fd);
}
/**
 * Transmits and receives data from CAN communication.
 *
 * Transmits contents of canTxBuffer and puts any received values to canRxBuffer.
 * Requires targetCanId and msgLen fields to be set before use.
 *
 * @return number of bytes received via CAN. If < 0, canalizator failed to respond.
 */
int Candle::transmitAndReceive()
{
    sprintf((char*)serialTxBuffer, "tr%.3d%.2d", targetCanId, msgLen);
    memcpy(&serialTxBuffer[7], canTxBuffer, msgLen);
    int serialTxLen = msgLen + 7;
    uart_transmit(fd, (char*)serialTxBuffer, serialTxLen);
    int serialRxLen = uart_receive(fd, (char*)serialRxBuffer, timeout);
    if( serialRxLen > 7)
        memcpy(canRxBuffer, &serialRxBuffer[7], serialRxLen - 7);
    receivedBytes = serialRxLen - 7;
    return receivedBytes;
}
/**
 * Receives data from CAN communication.
 *
 * Puts any received values to canRxBuffer.
 * Requires targetCanId and msgLen fields to be set before use.
 *
 * @return number of bytes received via CAN. If < 0, canalizator failed to respond.
 */
int Candle::receive()
{
    sprintf((char*)serialTxBuffer, "rx%.3d%.2d", targetCanId, msgLen);
    int serialTxLen = 7;
    uart_transmit(fd,(char*)serialTxBuffer, serialTxLen);
    int serialRxLen = uart_receive(fd, (char*)serialRxBuffer, timeout);
    if( serialRxLen > 7)
        memcpy(canRxBuffer, &serialRxBuffer[7], serialRxLen - 7);
    receivedBytes = serialRxLen - 7;
    return receivedBytes;
}
/**
 * Transmits and receives data from CAN communication.
 *
 * Transmits contents of canTxBuffer.
 * Requires targetCanId and msgLen fields to be set before use.
 *
 * @return true if data was send, false when not.
 */
bool Candle::transmit()
{
    sprintf((char*)serialTxBuffer, "tx%.3d%.2d", targetCanId, msgLen);
    memcpy(&serialTxBuffer[7], canTxBuffer, msgLen);
    int serialTxLen = msgLen + 7;
    if (uart_transmit(fd, (char*)serialTxBuffer, serialTxLen) > 0)
        return true;
    else  
        return false;
}

bool Candle::setRxTimeout(int timeoutMs)
{
    if(timeoutMs < 0 || timeoutMs > 10000)
    {
        std::cout << "Timeout out of range. Must be 1ms - 10000ms. Setting 100 ms." << std::endl;
        timeoutMs = 100;
    }
    sprintf((char*)serialTxBuffer, "cft%.4d", timeoutMs);
    uart_transmit(fd, (char*)serialTxBuffer, 7);
    if(uart_receive(fd,(char*)serialRxBuffer, timeoutMs) != 0)
    {
        timeout = timeoutMs;
        return true;
    }
    else
        return false;
    usleep(1000);
}

bool Candle::setCanSpeed(int canBaud)
{
    if(canBaud < 100000 || canBaud > 5000000)
    {
        std::cout << "CAN speed out of range. Must be 100kbps - 5Mbps. Setting 1Mbps." << std::endl;
        canBaud = CAN_DEFAULT_SPEED;
    }
    sprintf((char*)serialTxBuffer, "cfb%.7d", canBaud);
    uart_transmit(fd, (char*)serialTxBuffer, 10);
    if(uart_receive(fd,(char*)serialRxBuffer, timeout) != 0)
    {
        canSpeed = canBaud;
        return true;
    }
    else
        return false;
}

bool Candle::setMsgLen(int newMsgLen)
{
    if (newMsgLen < 0 || newMsgLen > 64)
    {
        std::cout << "Msg length out of range. Must be < 0 - 64 >." << std::endl;
        return false;
    }
    msgLen = newMsgLen;
    return true;
}

bool Candle::setTargetId(int newTargetId)
{
    if(newTargetId < 0 || newTargetId > 0x7ff)
    {
        std::cout << "Target Id out of range. Must be < 0 - 255 >." << std::endl;
        return false;
    }
    targetCanId = newTargetId;
    return true;
}

void Candle::setCanTx(const char* data, int len)
{
    if(len > 0 || len < 64)
        memcpy(canTxBuffer, data, len);
    else
        std::cout << "Could not fill Can Tx Buffer. Data len out of range. Must be < 0 - 64 >." << std::endl;
}
int Candle::getCanRx(char * rxData)
{
    if (receivedBytes > 0)
        memcpy(rxData, canRxBuffer, receivedBytes); 
    return receivedBytes;
}

bool Candle::isOk()
{
    if (fd > 0)
        return true;
    return false;
}
