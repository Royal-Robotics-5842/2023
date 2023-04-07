#pragma once

#include <numbers>

#include "frc/kinematics/DifferentialDriveOdometry.h"
#include "frc/drive/DifferentialDrive.h"
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc/smartdashboard/Field2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/ProfiledPIDSubsystem.h"
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

    frc::MotorControllerGroup mLeftSide{mLeftMaster, mLeftSlave};
    frc::MotorControllerGroup mRightSide{mRightMaster, mRightSlave};
    frc::DifferentialDrive mDrivetrain{mLeftSide, mRightSide};

    rev::SparkMaxRelativeEncoder mLeftEncoder{mLeftMaster.GetEncoder()};
    rev::SparkMaxRelativeEncoder mRightEncoder{mRightMaster.GetEncoder()};

    rev::SparkMaxPIDController mLeftController{mLeftMaster.GetPIDController()};
    rev::SparkMaxPIDController mRightController{mRightMaster.GetPIDController()};

    frc::ProfiledPIDController<units::degrees> turnPIDController{Constants::kTurnP, 0.0, 0.0, frc::TrapezoidProfile<units::degrees>::Constraints{60_deg_per_s, 60_deg_per_s_sq}};
    frc2::PIDController autobalancePIDController{Constants::kAutoP, 0.0, 0.0,};
    frc::ProfiledPIDController<units::meters> drivingPIDController{(32.483/10), 0.0, 0.0, frc::TrapezoidProfile<units::meters>::Constraints{1.5_mps, 1_mps_sq}};

    AHRS gyro{frc::SPI::Port::kMXP};

    //odometry
    frc::DifferentialDriveOdometry mOdometry{0_rad, 0_m, 0_m};
    frc::Pose2d mPose{{0_m, 0_m}, 0_rad};
    frc::Rotation2d mRotation{0_rad};
    frc::Field2d mField;

public:
    Drivetrain();

    void updatePose();

    void resetGyro();
    float getYaw();

    void enableBrake(bool mode);
    bool getMode();

    void drive(double left, double right);
    void driveDistance(units::meter_t dDistance);
    void cheesyDrive(double throttle, double wheel, bool quickTurn);
    void setVelocity(double left, double right);

    double getLeftVelocity();
    double getRightVelocity();

    void autobalance();
    void turnToAngle(double angle);


};
