#include "utils/calibration.hpp"

void perform(Md80*tested, Md80*brake)
{
    brake->setImpedance(0.0, 0.005, 0.0, 0.0, 0.0, 0.5);
    brake->setMode(md80_mode::IMPEDANCE_CTL);
    brake->enableMotor(true);

    tested->setImpedance(0.0, 0.5, 0.0, 50.0, 0.0, 0.5);
    tested->setMode(md80_mode::IMPEDANCE_CTL);
    tested->enableMotor(true);
}