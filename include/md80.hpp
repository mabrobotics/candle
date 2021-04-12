#ifndef _MD80_H_
#define _MD80_H_

#include "canalizator.hpp"

#include <cstdint>
#include <vector>

enum class md80_mode
{
    POSITION_PID = 0x01,
    VELOCITY_PID = 0x02,
    CASCADE_PID = 0x03,
    TORQUE_CTL = 0x04,
    IMPEDANCE_CTL = 0x05
};

class md80_regulator
{
public:
    md80_regulator(){kp = 0; ki = 0.0f; kd = 0.0f; posTarget = 0.0f, velTarget = 0.0f, iWindup = 0.0f; maxOutput = 0.0f; torqueCmd = 0.0f;};
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
    static Canalizator *pCan;
    int id;
    float position;
    float velocity;
    float torque;
    float currentLimit;
    uint16_t errorVector;
    md80_regulator positionReg;
    md80_regulator velocityReg;
    md80_regulator impedanceReg;
public:
    Md80(int id);
    static void initalize(Canalizator *can);
    static std::vector<int> sendPing(Canalizator*pCan, int idStart, int idEnd);
    void parseResponse(char rxBuffer[]);
    bool enableMotor(bool enable);
    bool setMode(md80_mode mode);
    bool setZeroPosition();
    bool setCurrentLimit(float newLimit);
    bool setNewConfig();
    bool setImpedance();
    bool setImpedance(float _kp, float _kd, float _posTarget, float _velTarget, float _torque, float _maxOutput);
    bool setPosition();
    bool setPosition(float kp, float ki, float kd, float ki_windup, float maxOutput, float posTarget);
    bool setVelocity();
    bool setVelocity(float kp, float ki, float kd, float ki_windup, float maxOutput, float velTarget);
    void printInfo();
    float getPosition(){return position;};
    float getVelocity(){return velocity;};
    float getTorque(){return torque;};
    int getId(){return id;};

    //DEBUG
    bool _changeId(uint16_t canId);
    bool _calibrate();
};

#endif
