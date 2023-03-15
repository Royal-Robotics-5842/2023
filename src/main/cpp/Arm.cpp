#include "Arm.h"

Arm::Arm()
{
    mArmMotor.RestoreFactoryDefaults();

    mArmMotor.SetSmartCurrentLimit(12);

    mArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    mArmEncoder.SetPosition(0);
    mArmEncoder.SetPositionConversionFactor(360/Constants::kArmGearRatio);

    mArmController.SetP(0.012496);
    mArmController.SetI(0);
    mArmController.SetD(0.0062782);
    mArmController.SetFF(0);
    mArmController.SetIZone(0);

    mSetPoint = -5; //stowed
    mConeMode = false; //cube mode
}



bool Arm::getMode()
{
    return mConeMode;
}

double Arm::getSetpoint()
{
    return mSetPoint;
}

double Arm::getPosition()
{
    return mArmEncoder.GetPosition();
}

void Arm::resetPosition()
{
    mArmEncoder.SetPosition(0);
    mArmEncoder.SetPositionConversionFactor(360/Constants::kArmGearRatio);
}

void Arm::toggleMode()
{
    //exclusive or/xor, one and only one true makes it true -- true ^ true = false, true ^ false = true
    mConeMode ^= true;
}

void Arm::brakeMode(bool input)
{
    if (input)
        mArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    else
        mArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

}

void Arm::setSpeed(double speed)
{
    //deadzone handling -- ensures inputs are still within the range [0, 1] even after discarding the inputs up to 0.05
    double scaledSpeed = (speed + (speed < 0 ? 0.1 : -0.1)) / (1 - 0.1);
    speed = (std::abs(speed) > 0.1) ? scaledSpeed : 0;

    mArmMotor.Set(speed*.70);
}

void Arm::setPosition(int preset)
{
    //if we're not in manual control mode
    if (preset != 0)
    {
        double position = 0;

        switch (preset)
        {
            case 1000: //stowed
                position = -5;
                break;
            case 2000: //mid goal
                //mConeMode ? heightIfCone : heightIfCube;
                position = mConeMode ? 75 : 80;
                break;
            case 3000: //high goal
                position = mConeMode ? 125 : 125;
                break;
            case 4000: //low goal
                position = mConeMode ? 33 : 38;
                break;
            default: //manual override, hold position
                position = getPosition();
                break;
        }
        
        m_goal = {(position * 1_deg), 0_deg_per_s};
        frc::TrapezoidProfile<units::degrees> profile{m_constraints, m_goal, m_setpoint};
        m_setpoint = profile.Calculate(kDt);

        std::cout << "Setpoint: " << m_setpoint.position.value() << std::endl; 
        
        mArmController.SetReference(m_setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition, 0);// mArmFF.Calculate(m_setpoint.position.value()*1_deg, 100_deg/1_s).value());
    }
}