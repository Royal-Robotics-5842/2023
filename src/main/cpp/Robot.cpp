// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Drivetrain.h"
#include "Arm.h"
#include "Intake.h"
#include "frc/XboxController.h"
#include <frc/Timer.h>
#include <fmt/core.h>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain drivetrain;
frc::XboxController controller{1};
Arm arm;
Intake intake;
frc::Timer t;

void Robot::RobotInit() 
{
//   m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
//   m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
//   frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotPeriodic() 
{
  drivetrain.updatePose();
}

void Robot::AutonomousInit() 
{
  // m_autoSelected = m_chooser.GetSelected();
  t.Restart();
}

void Robot::AutonomousPeriodic() 
{
  units::second_t autoTime = t.Get();
  int qwerty = (autoTime/1_s);
  cout<<qwerty<<endl;
  // if (m_autoSelected == kAutoNameCustom) {
  //   if (autoTime < 5_s)
  //   {
  //     drivetrain.drive(.4,.4);
  //     arm.brakeMode(true);
  //   }
  //   else
  //     drivetrain.enableBrake(true);
  // } 
  // else {
    if (autoTime < 1_s)
    {
      drivetrain.drive(.5,.5);
      intake.setSpeed(.7);
    }
    else if (autoTime < 3_s)
    {
      drivetrain.drive(0,0);
      arm.setPosition(2000);
    }
    else if (autoTime < 6_s)
      drivetrain.drive(-.4,-.4);
    else if (autoTime < 7_s)
    {
      drivetrain.drive(0,0); 
      intake.setSpeed(-.7); //positive for Cube, negative for Cone
    }
    else if (autoTime < 9_s)
    {
      intake.setSpeed(0);
      drivetrain.drive(.5,.5);
    }
    else if (autoTime < 13_s)
      arm.setPosition(1000);
    else
      arm.brakeMode(true);
  // }
  
  
}

void Robot::TeleopInit() {
  arm.brakeMode(true);
}

int armSetPoint = 0;
double lastPosition = 0;

void Robot::TeleopPeriodic() 
{
  arm.getMode() ? frc::SmartDashboard::PutString("Mode", "CUBE") : frc::SmartDashboard::PutString("Mode", "CONE");

  drivetrain.cheesyDrive(controller.GetLeftY(), -1*controller.GetRightX(), controller.GetRightBumper());

  //if we're in cone mode, intake exhausts a cube, so we need to invert intake controls when in cube mode so intake intakes always
  int sign = arm.getMode() ? -1 : 1;
  //intake overrides exhaust
  if (controller.GetRightTriggerAxis() != 0)
    intake.setSpeed(.5 * sign);
  else if (controller.GetLeftTriggerAxis() != 0)
    intake.setSpeed(-.5 * sign);
  else
    intake.setSpeed(0);

  //toggle between cone preset heights and cube preset heights -- default is cube
  //if limelight, also toggle vision mode when aiming between tape and tags
  if (controller.GetLeftBumperPressed())
    arm.toggleMode();

  if (controller.GetAButtonPressed())
    armSetPoint = 1;
  else if (controller.GetXButtonPressed())
    armSetPoint = 2;
  else if (controller.GetYButtonPressed())
    armSetPoint = 3;
  else if (controller.GetBButtonPressed())
    armSetPoint = 4;

  lastPosition = arm.getPosition();
  //manual control overrides position control
  if (std::abs(controller.GetRightY()) > 0.1)
  {
    //manual control setpoint needed to not revert to the previous position once manual controls stop
    armSetPoint = 0;
    arm.setSpeed(-1*controller.GetRightY());
  }
  else if (armSetPoint != 0)
  {
    //grab desired arm point on button press
    //this is sticky (we retain the setpoint until another input is received or manual control is initiated)
    if (controller.GetAButtonPressed())
      armSetPoint = 1; //stowed
    else if (controller.GetXButtonPressed())
      armSetPoint = 2; //mid goal
    else if (controller.GetYButtonPressed())
      armSetPoint = 3; //high goal
    else if (controller.GetBButtonPressed())
      armSetPoint = 4; //human player
    arm.setPosition(armSetPoint);
  }
  else
  {
    //arm.setPosition(lastPosition);
    arm.setSpeed(0);
  }
  std::cout << "Arm position: " << arm.getPosition() << endl;
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  arm.resetPosition();
  arm.brakeMode(false);
  arm.setSpeed(0);
}

//use test to set manually move the arm and zero it
void Robot::TestPeriodic() {
  std::cout << "Arm position: " << arm.getPosition() << endl;

  //Reset position once it's properly 0'd out
  if (controller.GetAButtonPressed())
    arm.resetPosition();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif