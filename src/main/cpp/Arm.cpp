#include "Arm.h"

Arm::Arm()
{
    mLeftArm.RestoreFactoryDefaults();
    mRightArm.RestoreFactoryDefaults();

    mLeftArm.SetSmartCurrentLimit(30);
    mRightArm.SetSmartCurrentLimit(30);

    mRightArm.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mLeftArm.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    mRightArm.SetInverted(true);

    mRightArm.Follow(mLeftArm, true);

    mArmEncoder.SetPosition(0);
    mArmEncoder.SetPositionConversionFactor(360/Constants::kArmGearRatio);

    mLeftArmController.SetP(0.052272);
    mLeftArmController.SetI(0);
    mLeftArmController.SetD(0);
    mLeftArmController.SetFF(0);
    mLeftArmController.SetIZone(0);

    mRightArmController.SetP(0.0052272);
    mRightArmController.SetI(0);
    mRightArmController.SetD(0);
    mRightArmController.SetFF(0);
    mRightArmController.SetIZone(0);

    mSetPoint = -110; //stowed
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

void Arm::resetEncoder()
{
    mArmEncoder.SetPosition(0);
}

void Arm::toggleMode()
{
    //exclusive or/xor, one and only one true makes it true -- true ^ true = false, true ^ false = true
    mConeMode ^= true;
}

void Arm::toggleConeMode()
{
    mConeMode = true;
}

void Arm::toggleCubeMode()
{
    mConeMode = false;
}
void Arm::brakeMode(bool input)
{
    if (input) {
        mLeftArm.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        mRightArm.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    } else {
        mLeftArm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        mRightArm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }

}

void Arm::setSpeed(double speed)
{
    //deadzone handling -- ensures inputs are still within the range [0, 1] even after discarding the inputs up to 0.05
    double scaledSpeed = (speed + (speed < 0 ? 0.1 : -0.1)) / (1 - 0.1);
    speed = (std::abs(speed) > 0.1) ? scaledSpeed : 0;

    if (speed < 0)
    {
      mLeftArm.Set(speed*0.5);
    }
    else
    {
      mLeftArm.Set(speed);
    }
    //mRightArm.Set(speed);
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
                position = 0;
                break;
            case 2000: //mid goal
                //mConeMode ? heightIfCone : heightIfCube;
                position = mConeMode ? -122.2 : -122.2;
                break;
            case 3000: //high goal
                position = mConeMode ? -145 : -145;
                break;
            case 4000: //human player
                position = mConeMode ? -33.4 : -33.4;
                break;
            case 5000: //low goal
                position = mConeMode ? -26.3 : -26.3;
                break;
            case 6000: //ground cube intake
            {
                position = mConeMode ? -98.47 :- 98.47;
                break;
                std::cout << "POSITION: "<< position;
            }
            /*default: //manual override, hold position
                position = getPosition();
                break;*/
            
            //return position;
        }
        
        m_goal = {(position * 1_deg), 0_deg_per_s};
        frc::TrapezoidProfile<units::degrees> profile{m_constraints, m_goal, m_setpoint};
        m_setpoint = profile.Calculate(kDt);
        //double output = mLeftArmPIDController.Calculate(mLeftArm.GetEncoder(), position);
        std::cout << "Setpoint: " << m_setpoint.position.value() << std::endl; 
        
        mLeftArmController.SetReference(m_setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition, 0);// mArmFF.Calculate(m_setpoint.position.value()*1_deg, 100_deg/1_s).value());
       // mRightArmController.SetReference
        //mRightArmController.SetReference(m_setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition, 0);
        
    }
}