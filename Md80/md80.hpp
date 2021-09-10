#ifndef _MD80_H_
#define _MD80_H_

#include "candle.hpp"

#include <cstdint>
#include <queue>
#include <list>
#include <thread>
#include <mutex>

namespace mab
{
    enum class Mode
    {
        POSITION_PID = 0x01,
        VELOCITY_PID = 0x02,
        CASCADE_PID = 0x03,
        TORQUE_CTL = 0x04,
        IMPEDANCE_CTL = 0x05
    };

    class RegulatorConfig
    {
    public:
        RegulatorConfig(){kp = 0; ki = 0.0f; kd = 0.0f; posTarget = 0.0f, velTarget = 0.0f, iWindup = 0.0f; maxOutput = 0.0f; torqueCmd = 0.0f;};
        float kp;
        float ki;
        float kd;
        float posTarget;
        float velTarget;
        float iWindup;
        float maxOutput;
        float torqueCmd;
    };


    class Md80
    {
    enum class MsgType : uint8_t
    {
        FLASH_LED = 0x00,
        ENABLE = 0x01,
        CONTROL_SELECT = 0x02,
        SET_ZERO_POSITION = 0x03,
        SET_MAX_CURRENT = 0x04,
        SET_POSITION_REG = 0x10,    //TODO
        SET_VELOCITY_REG = 0x11,    //TODO
        SET_IMPEDANCE_REG = 0x12,
        RESET_DRIVE = 0x13, //TODO
        GET_INFO = 0x14

        /* These are handled differenty than other msgs
        SET_CAN_CONFIG = 0x20,  
        SAVE_CAN_CONFIG = 0x21
        */
    };
    class Msg
    {
    public:
        Msg(int _id, Md80::MsgType _type){id = _id; msgType = _type;};
        int id;
        MsgType msgType;
        uint8_t data[32];
    };
    static mab::Candle *pCan;
    static int numOfDrives;
    static int commsFrequency;
    static std::mutex commsMutex;
    static std::thread commsThread;
    static void _commsThreadCallback();
    static void _commsPerform();
    static bool shouldStop;
    static std::queue<Msg> msgQueue;
    static std::list<Md80*> mdList;
    private:
        int id;
        float position;
        float velocity;
        float torque;
        float currentLimit;
        uint16_t errorVector;
        RegulatorConfig positionReg;
        RegulatorConfig velocityReg;
        RegulatorConfig impedanceReg;
        void _parseResponse(char rxBuffer[]);
        bool sendFlashLed();
        bool sendEnableMotor(bool enable);
        bool sendMode(mab::Mode mode);
        bool sendZeroPosition();
        bool sendCurrentLimit(float newLimit);
        bool sendGetInfo();
        bool sendImpedance(float _kp, float _kd, float _posTarget, float _velTarget, float _torque, float _maxOutput);
        bool sendPosition(float kp, float ki, float kd, float ki_windup, float posTarget, float maxOutput);
        bool sendVelocity(float kp, float ki, float kd, float ki_windup, float velTarget, float maxOutput);
        bool sendReset();
    public:
        //Creates an instance of Md80 with selected can id
        Md80(int id);
        ~Md80();
        // Passes a pointer to CANdle object, and uses it to communicate with Md80s
        static void initalize(mab::Candle *can);
        //Shuts down communication with Md80s
        static void deinitalize();

        // Sends CAN ping command to all drives with IDs from idStart to idEnd, prints list of drives that responded
        static std::vector<int> sendPing(mab::Candle*pCan, int idStart, int idEnd);
        
        // Prints last received Position, Velocity, Torque and Error Vector
        void printInfo();
        
        void flashLed();
        void enable();
        void disable();
        void setControlMode(mab::Mode mode);
        void setZeroPosition();
        void setCurrentLimit(float currentLimit);
        void setImpedanceController(float kp, float kd, float positionTarget, float velocityTarget, float torque, float maxOutput);
        void setPositionController(float kp, float ki, float kd, float iWindup, float maxOutput, float positionTarget);
        void setVelocityController(float kp, float ki, float kd, float iWindup, float maxOutput, float velocityTarget);
        
        // Returns last received Position
        float getPosition(){return position;};
        // Returns last received Velocity
        float getVelocity(){return velocity;};
        // Returns last received Torque
        float getTorque(){return torque;};
        // Returns drives Id
        int getId(){return id;};

        //CONFIGURATION FRAMES - for may brake the drive is used incorrectly
        bool _setWatchdogStop();
        bool _setNewConfig();
        bool _setNewCanId(uint16_t canId);
        bool _setCalibration();
    };
}
#endif
