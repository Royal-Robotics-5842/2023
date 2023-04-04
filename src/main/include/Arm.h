#pragma once

#include "Constants.h"
#include "rev/CANSparkMax.h"
#include "frc/controller/ArmFeedforward.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include "units/acceleration.h"
#include "units/length.h"
#include "units/time.h"
#include "units/velocity.h"
#include "units/voltage.h"

#include <fmt/core.h>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>


class Arm
{
    rev::CANSparkMax mLeftArm{31, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax mRightArm{32, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder mArmEncoder{mLeftArm.GetEncoder()};
    rev::SparkMaxPIDController mLeftArmController{mLeftArm.GetPIDController()};
    rev::SparkMaxPIDController mRightArmController{mRightArm.GetPIDController()};
    //frc2::PIDController mLeftArmPIDController{0.0052272, 0.0, 0.0,};
    frc::ArmFeedforward mArmFF{Constants::kArmS, Constants::kArmG, Constants::kArmV};

    bool mConeMode;
    double mSetPoint;

    public:
    Arm();

    bool getMode();
    double getSetpoint();
    double getPosition();
    void resetEncoder();

    void toggleMode();
    void toggleConeMode();
    void toggleCubeMode();
    void setSpeed(double speed);
    void setPosition(int preset);

    void brakeMode(bool input);

    static constexpr units::second_t kDt = 20_ms;
    frc::TrapezoidProfile<units::degrees>::Constraints m_constraints{80_deg_per_s, 100_deg_per_s_sq};
    frc::TrapezoidProfile<units::degrees>::State m_goal;
    frc::TrapezoidProfile<units::degrees>::State m_setpoint;
};