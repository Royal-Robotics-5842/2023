#include "Drivetrain.h"
#include <iostream>

Drivetrain::Drivetrain()
{
    //ensure no side effects mess with motors
    mLeftMaster.RestoreFactoryDefaults();
    mLeftSlave.RestoreFactoryDefaults();
    mRightMaster.RestoreFactoryDefaults();
    mRightSlave.RestoreFactoryDefaults();

    mLeftMaster.SetSmartCurrentLimit(80);
    mLeftSlave.SetSmartCurrentLimit(80);
    mRightMaster.SetSmartCurrentLimit(80);
    mRightSlave.SetSmartCurrentLimit(80);

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

    turnPIDController.EnableContinuousInput(-180.0_deg, 180.0_deg);
    turnPIDController.SetTolerance(1_deg, 1_deg_per_s);


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

void Drivetrain::resetGyro()
{
    gyro.ZeroYaw();
}

bool Drivetrain::getMode()
{
    if (mLeftMaster.GetIdleMode() == rev::CANSparkMax::IdleMode::kBrake){
        return true;
    } else if (mLeftMaster.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast){
        return false;
    }
}

float Drivetrain::getYaw()
{
    return gyro.GetYaw();
}

void Drivetrain::enableBrake(bool mode)
{
    switch(mode){
        case true:
            mLeftMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            mLeftSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            mRightMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
            mRightSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        case false:
            mLeftMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            mLeftSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            mRightMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
            mRightSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }
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

void Drivetrain::autobalance()
{
    bool autoBalance = false;
    static const double kOffBalanceThresholdDegrees = 5.0;
    static const double kOnBalanceThresholdDegrees  = 2.5;
    double pitchAngleDegrees = gyro.GetPitch();
      
    if (autoBalance != true && (std::abs(pitchAngleDegrees) >= std::abs(kOffBalanceThresholdDegrees))) {
        autoBalance = true;
    }

    else if (autoBalance != false && (std::abs(pitchAngleDegrees) <= std::abs(kOnBalanceThresholdDegrees))) {
        autoBalance = false;
    }
  // control drive system automatically, driving in reverse direction of pitch angle, w/ magnitude based on angle
    if (autoBalance = true) {
        double pitchAngleRadians = pitchAngleDegrees*(numbers::pi/180);
        double pitchcorrect = sin(pitchAngleRadians)*-1;
        drive(pitchcorrect, pitchcorrect);
    }
  sleep(.005); // wait 5ms to avoid hogging CPU cycles
}

void Drivetrain::turnToAngle(double angle) 
{
    turnPIDController.SetGoal({angle*1_deg, 0_deg_per_s});
    double turnToAnglePower = turnPIDController.Calculate(gyro.GetYaw()*1_deg);
    std::cout << turnToAnglePower << endl;
    mLeftController.SetReference(-turnToAnglePower, rev::CANSparkMax::ControlType::kDutyCycle);
    mRightController.SetReference(turnToAnglePower, rev::CANSparkMax::ControlType::kDutyCycle);
}