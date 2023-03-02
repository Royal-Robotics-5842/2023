#include "Intake.h"

Intake::Intake()
{
    mIntakeMotor.ConfigFactoryDefault();

    mIntakeMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void Intake::setSpeed(double speed)
{
    mIntakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}