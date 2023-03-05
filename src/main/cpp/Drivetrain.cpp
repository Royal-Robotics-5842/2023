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
    mLeftMaster.SetInverted(false);
    mLeftSlave.SetInverted(false);

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
    mLeftMaster.Set(left);
    mRightMaster.Set(right);
}

void Drivetrain::cheesyDrive(double throttle, double wheel, bool isQuickTurn)
{
    
    //throttle = (throttle + (throttle < 0 ? 0.08 : -0.08)) / (1 - 0.08);
    //wheel = (wheel + (wheel < 0 ? 0.08 : -0.08)) / (1 - 0.08);

    double scaledThrottle = (throttle + (throttle < 0 ? 0.08 : -0.08)) / (1 - 0.08);
    throttle = (std::abs(throttle) > 0.05) ? scaledThrottle : 0;

    double scaledWheel = (wheel + (wheel < 0 ? 0.08 : -0.08)) / (1 - 0.08);
    wheel = (std::abs(wheel) > 0.08) ? scaledWheel : 0;

    double leftOutput, rightOutput;

    //scales wheel component to make it more smooth
    double wheelNonLinearity = 0.025;
    double denominator = std::sin(numbers::pi / 2.0 * wheelNonLinearity);
    wheel = std::sin(numbers::pi / 2.0 * wheelNonLinearity * wheel) / denominator;
    wheel = std::sin(numbers::pi / 2.0 * wheelNonLinearity * wheel) / denominator;

    double negInertia = 4 * (wheel - mOldWheel);
    mOldWheel = wheel;
    mNegInertiaAccumulator += negInertia;
    wheel = wheel + mNegInertiaAccumulator;
    if (mNegInertiaAccumulator > 1) {
        mNegInertiaAccumulator -= 1;
    } else if (mNegInertiaAccumulator < -1) {
        mNegInertiaAccumulator += 1;
    } else {
        mNegInertiaAccumulator = 0;
    }

    double overPower;
    if (isQuickTurn)
    {
        if (std::abs(throttle) < 0.5) 
            mQuickStopAccumulator = (1 - .1) * mQuickStopAccumulator
                + .1 * std::min(1.0, std::max(-1.0, wheel)) * 5;
        overPower = 1.0;
    } 
    else 
    {
        overPower = 0.0;
        wheel = std::abs(throttle) * wheel * 0.65 - mQuickStopAccumulator;
        if (mQuickStopAccumulator > 1)
            mQuickStopAccumulator -= 1;
        else if (mQuickStopAccumulator < -1)
            mQuickStopAccumulator += 1;
        else
            mQuickStopAccumulator = 0.0;
    }

    rightOutput = throttle - wheel, leftOutput = throttle + wheel;

    if (leftOutput > 1.0) {
        rightOutput -= overPower * (leftOutput - 1.0);
        leftOutput = 1.0;
    } else if (rightOutput > 1.0) {
        leftOutput -= overPower * (rightOutput - 1.0);
        rightOutput = 1.0;
    } else if (leftOutput < -1.0) {
        leftOutput = -1.0;
    } else if (rightOutput < -1.0) {
        leftOutput += overPower * (-1.0 - rightOutput);
        rightOutput = -1.0;
    }

    double scalingFactor = std::max(1.0, abs(throttle));
    //cout << "left: " << leftOutput*3.8/scalingFactor << " right: " << rightOutput*3.8/scalingFactor << endl;
    //tankDrive(leftOutput/scalingFactor, rightOutput/scalingFactor);
    drivetrain.TankDrive(leftOutput/scalingFactor, rightOutput/scalingFactor);
    drivetrain.Feed();
}

void Drivetrain::setVelocity(double left, double right)
{
    mLeftController.SetReference(left, rev::CANSparkMax::ControlType::kVelocity);
    mRightController.SetReference(right, rev::CANSparkMax::ControlType::kVelocity);
}