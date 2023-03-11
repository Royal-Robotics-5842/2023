#include "Drivetrain.h"

Drivetrain::Drivetrain()
{
    //ensure no side effects mess with motors
    mLeftMaster.RestoreFactoryDefaults();
    mLeftSlave.RestoreFactoryDefaults();
    mRightMaster.RestoreFactoryDefaults();
    mRightSlave.RestoreFactoryDefaults();

    mLeftMaster.SetSmartCurrentLimit(30);
    mLeftSlave.SetSmartCurrentLimit(30);
    mRightMaster.SetSmartCurrentLimit(30);
    mRightSlave.SetSmartCurrentLimit(30);

    mLeftMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mLeftSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mRightMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mRightSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    //right motors are opposite left motors, so must spin other direction
    mLeftMaster.SetInverted(true);
    mLeftSlave.SetInverted(true);
    mRightMaster.SetInverted(false);
    mRightSlave.SetInverted(false);

    //set slaves to follow/mimic masters
    mLeftSlave.Follow(mLeftMaster);
    mRightSlave.Follow(mRightMaster);

    mLeftEncoder.SetPosition(0);
    mLeftEncoder.SetPositionConversionFactor((Constants::kWheelDiameter*numbers::pi)/Constants::kLowGearRatio);
    mLeftEncoder.SetVelocityConversionFactor((Constants::kWheelDiameter*numbers::pi)/Constants::kLowGearRatio/60);

    mRightEncoder.SetPosition(0);
    mRightEncoder.SetPositionConversionFactor((Constants::kWheelDiameter*numbers::pi)/Constants::kLowGearRatio);
    mRightEncoder.SetVelocityConversionFactor((Constants::kWheelDiameter*numbers::pi)/Constants::kLowGearRatio/60);

    mLeftController.SetP(Constants::kP);
    mLeftController.SetFF(Constants::kF);

    mRightController.SetP(Constants::kP);
    mRightController.SetFF(Constants::kF);

    gyro.ZeroYaw();
    mField.SetRobotPose(mPose);
    frc::SmartDashboard::PutData(&mField);
}

void Drivetrain::updatePose()
{
    mRotation = gyro.GetRotation2d();
    mPose = mOdometry.Update(mRotation, mLeftEncoder.GetPosition()*1_m, mRightEncoder.GetPosition()*1_m);
    mField.SetRobotPose(mPose);
}

void Drivetrain::drive(double left, double right)
{
   mDrivetrain.TankDrive(left, right);
}

void Drivetrain::cheesyDrive(double throttle, double wheel, bool isQuickTurn)
{
    
     //deadzone handling -- ensures inputs are still within the range [0, 1] even after discarding the inputs up to 0.05
     double scaledThrottle = (throttle + (throttle < 0 ? 0.075 : -0.075)) / (1 - 0.075);
     throttle = (std::abs(throttle) > 0.075) ? scaledThrottle : 0;

     double scaledWheel = (wheel + (wheel < 0 ? 0.075 : -0.075)) / (1 - 0.075);
     wheel = (std::abs(wheel) > 0.075) ? scaledWheel : 0;

     // Apply a sin function that's scaled to make it feel better.
     double wheelNonLinearity = 0.5;
     double denominator = std::sin(numbers::pi / 2.0 * wheelNonLinearity);

     if (!isQuickTurn) {
         wheel = std::sin(numbers::pi / 2.0 * wheelNonLinearity * wheel);
         wheel = std::sin(numbers::pi / 2.0 * wheelNonLinearity * wheel);
         wheel = wheel / (denominator * denominator) * std::abs(throttle);
     }

     wheel *= 1;

     double rightOutput = throttle - wheel, leftOutput = throttle + wheel;
     double scalingFactor = std::max(1.0, abs(throttle));

     drive(leftOutput/scalingFactor, rightOutput/scalingFactor);
 }

void Drivetrain::setVelocity(double left, double right)
{
    mLeftController.SetReference(left, rev::CANSparkMax::ControlType::kVelocity);
    mRightController.SetReference(right, rev::CANSparkMax::ControlType::kVelocity);
}

double Drivetrain::getLeftVelocity()
{
    return mLeftEncoder.GetVelocity();
}

double Drivetrain::getRightVelocity()
{
    return mRightEncoder.GetVelocity();
}