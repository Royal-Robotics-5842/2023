#pragma once

#include "ctre/phoenix/motorcontrol/can/VictorSPX.h"

class Intake
{
    ctre::phoenix::motorcontrol::can::VictorSPX mIntakeMotor{21};

    public:
    Intake();
    void setSpeed(double speed);
};