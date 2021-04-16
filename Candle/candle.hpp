#ifndef _CANDLE_H_
#define _CANDLE_H_

#include <string>
#include <Python.h>
namespace mab
{
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
        
        //static int fd2;
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
        int isOk2();

        void registerValue()
        {
          //fd2 = fd;
        }
        static PyObject* isOk(PyObject* self, PyObject* args);
        //int giveRand(int value);
        static PyObject* giveRandom(PyObject* self, PyObject* args);
    };
}
#endif