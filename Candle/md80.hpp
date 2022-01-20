#include "mab_types.hpp"

#include <cstdint>
#include <cmath>

using namespace mab;

class Md80
{
private:
    uint16_t canId;

    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    uint16_t errorVector = 0;

    Md80Mode_E controlMode = Md80Mode_E::IDLE;
    float positionTarget = 0.0f;
    float velocityTarget = 0.0f;
    float torqueSet = 0.0f;
    float maxTorque = 1.8f;
    float maxVelocity = 300.0f;
    RegPid_t velocityRegulator;
    RegPid_t positionRegulator;
    RegImpedance_t impedanceRegulator;

    bool regulatorsAdjusted = false;
    StdMd80CommandFrame_t commandFrame;
public:
    Md80(uint16_t canID);
    ~Md80();
    void setPositionRegulator(float kp, float ki, float kd, float iWindup);
    void setVelocityRegulator(float kp, float ki, float kd, float iWindup);
    void setImpedanceRegulator(float kp, float kd, float torque);
    void updateCommandFrame();
    void updateResponseData();
    
    //advanced setters
    void setMaxTorque(float maxTorque);
    void setMaxVelocity(float maxVelocity);
    void setControlMode(Md80Mode_E mode);
    
    //simple setters
    void setTargetPosition(float target)    {positionTarget = target; };
    void setTargetVelocity(float target)    {velocityTarget = target; };
    void setTorque(float target)            {torqueSet = target; };
    
    //getters
    uint16_t getErrorVector()               {return errorVector; };
    StdMd80CommandFrame_t getCommandFrame() {return commandFrame;};
};

