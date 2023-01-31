#include "Drivetrain.h"

Drivetrain::Drivetrain()
{
    mLeftMaster.RestoreFactoryDefaults();
    mRightMaster.RestoreFactoryDefaults();

    mRightMaster.SetInverted(true);

    mLeftEncoder.SetPosition(0);
    mLeftEncoder.SetPositionConversionFactor((Constants::kWheelDiameter*numbers::pi)/Constants::kGearRatio);
    mLeftEncoder.SetVelocityConversionFactor((Constants::kWheelDiameter*numbers::pi)/Constants::kGearRatio/60);

    mRightEncoder.SetPosition(0);
    mRightEncoder.SetPositionConversionFactor((Constants::kWheelDiameter*numbers::pi)/Constants::kGearRatio);
    mRightEncoder.SetVelocityConversionFactor((Constants::kWheelDiameter*numbers::pi)/Constants::kGearRatio/60);

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
    mLeftMaster.Set(left);
    mRightMaster.Set(right);
}

void Drivetrain::setVelocity(double left, double right)
{
    mLeftController.SetReference(left, rev::CANSparkMax::ControlType::kVelocity);
    mRightController.SetReference(right, rev::CANSparkMax::ControlType::kVelocity);
}