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
    void resetPosition();

    void toggleMode();
    void setSpeed(double speed);
    void setPosition(int preset);

    void brakeMode(bool input);

    static constexpr units::second_t kDt = 20_ms;
    frc::TrapezoidProfile<units::degrees>::Constraints m_constraints{100_deg_per_s, 120_deg_per_s_sq};
    frc::TrapezoidProfile<units::degrees>::State m_goal;
    frc::TrapezoidProfile<units::degrees>::State m_setpoint;
};