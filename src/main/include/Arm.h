#pragma once

#include "Constants.h"
#include "rev/CANSparkMax.h"
#include "frc/controller/ArmFeedforward.h"

class Arm
{
    rev::CANSparkMax mArmMotor{31, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder mArmEncoder{mArmMotor.GetEncoder()};
    rev::SparkMaxPIDController mArmController{mArmMotor.GetPIDController()};

    frc::ArmFeedforward mArmFF{Constants::kArmS, Constants::kArmG, Constants::kArmV};

    bool mConeMode;
    double mSetPoint;

    public:
    Arm();

    bool getMode();
    double getSetpoint();
    double getPosition();

    void toggleMode();
    void setSpeed(double speed);
    void setPosition(int preset);
};