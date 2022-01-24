#pragma once

#include "mab_types.hpp"

#include <cstdint>

namespace mab
{
    class Md80
    {
    public:
        Md80(uint16_t canID);
        ~Md80();
        void setPositionRegulator(float kp, float ki, float kd, float iWindup);
        void setVelocityRegulator(float kp, float ki, float kd, float iWindup);
        void setImpedanceRegulator(float kp, float kd);
        void updateCommandFrame();
        void updateResponseData(StdMd80ResponseFrame_t*_responseFrame);
        
        //simple setters
        void setMaxTorque(float maxTorque);
        void setMaxVelocity(float maxVelocity);
        void setControlMode(Md80Mode_E mode);
        void setTargetPosition(float target);
        void setTargetVelocity(float target);
        void setTorque(float target);
        
        //getters
        uint16_t getErrorVector();
        StdMd80CommandFrame_t getCommandFrame();
        uint16_t getId();
        float getPosition();
        float getVelocity();
        float getTorque();

    };
}
