#include "candle.hpp"
#include "uart.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include "unistd.h"

#define CAN_DEFAULT_SPEED 1000000
#define CANDLE_TRANSMIT 't'
#define CANDLE_RECEIVE 'r'
#define CANDLE_TRANSMIT_AND_RECEIVE 'x'

namespace mab
{
    Candle::Candle()
    {

    }

    Candle::Candle(std::string canalizatorDev, int canBaudrate = 1000000)
    {
        fd = uart_init(&canalizatorDev[0], 100);
        if (fd < 0)
        {
            std:: cout << "Failed to open port " << canalizatorDev << std::endl;
            return;
        }
        std::string setSerialCommand = "setserial " + canalizatorDev + " low_latency";
        if (system(setSerialCommand.c_str()) != 0)
        {
            std:: cout << "Could not execute command '" << setSerialCommand <<"'. Communication in low-speed mode." << std::endl;
            return;
        }
        setRxTimeout(1000);
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
    int Candle::receive()
    {
        return transfer(false, true);
    }
    bool Candle::transmit()
    {
        if(transfer(true, false) >= 0)
            return true;
        return false;
    }
    int Candle::transmitAndReceive()
    {
        return transfer(true, true);
    }

    int Candle::transfer(bool tx, bool rx)
    {
        if(tx)
            serialTxBuffer[0] = CANDLE_TRANSMIT;
        if(rx)
            serialTxBuffer[0] = CANDLE_RECEIVE;
        if(tx && rx)
            serialTxBuffer[0] = CANDLE_TRANSMIT_AND_RECEIVE;
        int serialTxLen = 1;
        if(tx)
        {
            *(uint16_t*)&serialTxBuffer[1] = targetCanId;
            serialTxBuffer[3] = msgLen;
            memcpy(&serialTxBuffer[4], canTxBuffer, msgLen);
            serialTxLen = msgLen + 4;
            if(!rx)
                receivedBytes = 0;
        }
        uart_transmit(fd, (char*)serialTxBuffer, serialTxLen);
        int serialRxLen = uart_receive(fd, (char*)serialRxBuffer, timeout);
        if(rx)
        {
            if( serialRxLen > 4)
                memcpy(canRxBuffer, &serialRxBuffer[4], serialRxLen - 4);
            receivedBytes = serialRxLen - 4;
        }
        return receivedBytes;
    }

    bool Candle::setRxTimeout(int timeoutUs)
    {
        if(timeoutUs < 10 || timeoutUs > 100000)
        {
            std::cout << "Timeout out of range. Must be 10us - 100000us. Setting 1000 us." << std::endl;
            timeoutUs = 1000;
        }
        serialTxBuffer[0] = 'c';
        serialTxBuffer[1] = 't';
        *(uint32_t*)&serialTxBuffer[2] = timeoutUs;
        uart_transmit(fd, (char*)serialTxBuffer, 2+4);
        if(uart_receive(fd,(char*)serialRxBuffer, timeout) != 0)
        {
            timeout = timeoutUs/1000;
            return true;
        }
        else
            return false;
    }

    bool Candle::setCanSpeed(int canBaud)
    {
        if(canBaud != 1000000 && canBaud != 2500000 && canBaud != 5000000)
        {
            std::cout << "CAN speed out of range. Must be 1M, 2.5M or 5M. Setting 1Mbps." << std::endl;
            canBaud = CAN_DEFAULT_SPEED;
        }
        serialTxBuffer[0] = 'c';
        serialTxBuffer[1] = 'b';
        *(uint32_t*)&serialTxBuffer[2] = canBaud;
        uart_transmit(fd, (char*)serialTxBuffer, 2+4);
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


    
}

