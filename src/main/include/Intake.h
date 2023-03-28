#pragma once

#include "rev/CANSparkMax.h"

class Intake
{
    rev::CANSparkMax mLeftIntake{21, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mRightIntake{22, rev::CANSparkMax::MotorType::kBrushless};

    public:
    Intake();
    void setSpeed(double speed);
};