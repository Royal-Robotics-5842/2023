#pragma once

#include <numbers>

#include "frc/kinematics/DifferentialDriveOdometry.h"
#include "frc/smartdashboard/Field2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include "Constants.h"

using namespace std;

class Drivetrain
{
    rev::CANSparkMax mLeftMaster{11, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mLeftSlave{13, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mRightMaster{12, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax mRightSlave{14, rev::CANSparkMax::MotorType::kBrushless};

    rev::SparkMaxRelativeEncoder mLeftEncoder{mLeftMaster.GetEncoder()};
    rev::SparkMaxRelativeEncoder mRightEncoder{mRightMaster.GetEncoder()};

    rev::SparkMaxPIDController mLeftController{mLeftMaster.GetPIDController()};
    rev::SparkMaxPIDController mRightController{mRightMaster.GetPIDController()};

    AHRS gyro{frc::SPI::Port::kMXP};

    //odometry
    frc::DifferentialDriveOdometry mOdometry{0_rad, 0_m, 0_m};
    frc::Pose2d mPose{{0_m, 0_m}, 0_rad};
    frc::Rotation2d mRotation{0_rad};
    frc::Field2d mField;

public:
    Drivetrain();

    void updatePose();

    void drive(double left, double right);
    void cheesyDrive(double throttle, double wheel, bool quickTurn);
    void setVelocity(double left, double right);

};
