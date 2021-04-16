#ifndef _MD80_H_
#define _MD80_H_

#include "candle.hpp"

#include <cstdint>
#include <vector>

namespace mab
{
    namespace md80
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
        private:
            static mab::Candle *pCan;
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
        public:
            //Creates an instance of Md80 with selected can id
            Md80(int id);
            // Passes a pointer to CANdle class to be used for communication with Md80s
            static void initalize(mab::Candle *can);
            // Sends CAN ping command to all drives with IDs from idStart to idEnd, prints list of drives that responded
            static std::vector<int> sendPing(mab::Candle*pCan, int idStart, int idEnd);
            // Sends CAN command to enable/disable the motor
            bool enableMotor(bool enable);
            // Sends CAN command to select control mode 
            bool setMode(mab::md80::Mode mode);
            // Sends CAN command to set absolute position to zero at the current position
            bool setZeroPosition();
            // Sends CAN command to set up a current limiter
            bool setCurrentLimit(float newLimit);
            // Sends CAN command with current Impedance Controller config
            bool setImpedance();
            // Sends CAN command to update Impedance Controller config
            bool setImpedance(float _kp, float _kd, float _posTarget, float _velTarget, float _torque, float _maxOutput);
            // Sends CAN command with current Position Controller config
            bool setPosition();
            // Sends CAN command to update Position Controller Config
            bool setPosition(float kp, float ki, float kd, float ki_windup, float posTarget, float maxOutput);
            // Sends CAN command with curret Velocity Controller onfig
            bool setVelocity();
            // Sends CAN command update Velocity Controller config
            bool setVelocity(float kp, float ki, float kd, float ki_windup, float velTarget, float maxOutput);
            // Prints last received Position, Velocity, Torque and Error Vector
            void printInfo();
            // Returns last received Position
            float getPosition(){return position;};
            // Returns last received Velocity
            float getVelocity(){return velocity;};
            // Returns last received Torque
            float getTorque(){return torque;};
            // Returns drives Id
            int getId(){return id;};

            //CONFIGURATION FRAMES - for may brake the drive is used incorrectly
            bool _setNewConfig();
            bool _changeId(uint16_t canId);
            bool _calibrate();
        };
    }
}
#endif
