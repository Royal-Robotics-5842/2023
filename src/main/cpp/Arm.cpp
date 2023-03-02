#include "Arm.h"

Arm::Arm()
{
    mArmMotor.RestoreFactoryDefaults();

    mArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    mArmEncoder.SetPosition(0);
    mArmEncoder.SetPositionConversionFactor(360/75);

    mArmController.SetP(0.012819);
    mArmController.SetI(0);
    mArmController.SetD(0.00362);
    mArmController.SetFF(0);
    mArmController.SetIZone(0);

    mSetPoint = -10; //stowed
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

void Arm::toggleMode()
{
    //exclusive or/xor, one and only one true makes it true -- true ^ true = false, true ^ false = true
    mConeMode ^= true;
}

void Arm::setSpeed(double speed)
{
    //deadzone handling -- ensures inputs are still within the range [0, 1] even after discarding the inputs up to 0.05
    double scaledSpeed = (speed + (speed < 0 ? 0.1 : -0.1)) / (1 - 0.1);
    speed = (std::abs(speed) > 0.1) ? scaledSpeed : 0;

    mArmMotor.Set(speed*0.3333);
}

void Arm::setPosition(int preset)
{
    //if we're not in manual control mode
    if (preset != 0)
    {
        double position = 0;

        switch (preset)
        {
            case 1: //stowed
                position = -10;
                break;
            case 2: //mid goal
                //mConeMode ? heightIfCone : heightIfCube;
                position = mConeMode ? 50 : 40;
                break;
            case 3: //high goal
                position = mConeMode ? 100 : 80;
                break;
            case 4: //human player
                position = mConeMode ? 80 : 70;
                break;
            default: //manual override, hold position
                position = getPosition();
                break;
        }
        
        mArmController.SetReference(position, rev::CANSparkMax::ControlType::kPosition, 0, mArmFF.Calculate(position*1_deg, 100_deg/1_s).value());
    }
}