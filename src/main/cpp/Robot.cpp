// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Drivetrain.h"
#include "Arm.h"
#include "Intake.h"
#include "Pneumatics.h"
#include "frc/XboxController.h"

#include <fmt/core.h>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain drivetrain;
frc::XboxController controller{1};
Arm arm;
Intake intake;
frc::Timer t;
Pneumatics Pneumatic;

int auton;

void Robot::RobotInit() 
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  arm.resetEncoder();

}

void Robot::RobotPeriodic() 
{
  drivetrain.updatePose();
}

void Robot::AutonomousInit() 
{
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);
  int i = 1;
  t.Restart();
  while(i == 1)
  {
    if (controller.GetYButton())
    {
        auton = 1;
        break;
    }
    else if (controller.GetBButton()) 
    {
        auton = 2;
        break;
    } 
    else if (controller.GetAButtonPressed()) 
    {
        auton = 3;
        break;
    } 
    else if (controller.GetXButtonPressed()) 
    {
        auton = 4;
        break;
    }
    else
    {
        auton = 1;
        break;
    }
    // t.Start();
    i++;
    std::cout << auton << endl;
  }
}

void Robot::AutonomousPeriodic() 
{
  units::second_t autoTime = t.Get();
  //double time2 = autoTime/1_s;
  
  switch (auton)
  {  
  /* 
  case (1): //21 Point
    //std::cout << "TIME: " << time2 <<endl;

    if (autoTime < 4.8_s)
    {
      arm.setPosition(3000);
      drivetrain.driveDistance(-30_in);
      arm.brakeMode(true);
    }
    else if (autoTime < 5.8_s){
      intake.setSpeed(-0.8);
    }
    else if (autoTime < 9_s)
    {
      arm.setPosition(1000);
      drivetrain.driveDistance(350_in);
      intake.setSpeed(0);
    }
    else if (autoTime < 9.1_s)
    {
      drivetrain.enableBrake(true);
    }
    else if (autoTime < 12_s)
    {
      drivetrain.drive(-0.6, -0.6);
    }
    else if (autoTime < 15_s)
      drivetrain.autobalance();
      */
  /*  
  case (1): //18 Poiint
  if (autoTime < 6_s)
    {
      arm.setPosition(3000);
      drivetrain.driveDistance(-30_in);
      arm.brakeMode(true);
    }
    else if (autoTime < 7_s){
      intake.setSpeed(-0.8);
    }
   else if (autoTime < 9_s)
    {
      arm.setPosition(1000);
      drivetrain.driveDistance(75_in);
    }
    else if (autoTime < 15_s)
    {
      drivetrain.autobalance();
  */
  
  case (1): //Side Piece and +3
    if (autoTime < 6_s)
    {
      arm.setPosition(3000);
      arm.brakeMode(true);
    }
    else if (autoTime < 9_s){
      drivetrain.driveDistance(-50_in);
    }
    else if (autoTime < 9.25_s)
    {
      intake.setSpeed(-0.8);
    }
    else if (autoTime < 13_s)
    {
      drivetrain.driveDistance(270_in);
      arm.setPosition(1000);
    }
    /*
    case (4): //High Goal and Move -- CUBE
    if (autoTime < 4_s) 
	    arm.setPosition(3000);
    else if (autoTime < 6_s)
      intake.setSpeed(0.7);
    else if (autoTime < 8_s)
      arm.setPosition(1000);
    else if (autoTime < 11_s)
      drivetrain.drive(.7, .7);
    else if (autoTime < 15_s)
      drivetrain.enableBrake(true);
    break;
    */
  }
  }

void Robot::TeleopInit() {
  arm.brakeMode(true);
  drivetrain.enableBrake(false);
}

int armSetPoint = 0;
double lastPosition = 0;

void Robot::TeleopPeriodic() 
{
  arm.getMode() ? frc::SmartDashboard::PutString("Mode", "CUBE") : frc::SmartDashboard::PutString("Mode", "CONE");
  drivetrain.getMode() ? frc::SmartDashboard::PutString("Drivetrain Mode", "BRAKE") : frc::SmartDashboard::PutString("Drivetrain Mode", "COAST");

  if(controller.GetRightStickButtonPressed()){
    if(drivetrain.getMode()){
      drivetrain.enableBrake(0);
    } else if(!drivetrain.getMode()){
      drivetrain.enableBrake(1);
    }
  }

  drivetrain.cheesyDrive(controller.GetLeftY(), -1*controller.GetRightX(), controller.GetRightBumper());

  //if we're in cone mode, intake exhausts a cube, so we need to invert intake controls when in cube mode so intake intakes always
  int sign = arm.getMode() ? -1 : 1;
  //intake overrides exhaust
  if (controller.GetRightTriggerAxis() != 0)
    intake.setSpeed(.7 * sign);
  else if (controller.GetLeftTriggerAxis() != 0)
    intake.setSpeed(-.7 * sign);
  else
    intake.setSpeed(0);

  //toggle between cone preset heights and cube preset heights -- default is cube
  //if limelight, also toggle vision mode when aiming between tape and tags
  /*
  if (controller.GetPOV() == 90)
    arm.toggleConeMode();
  if (controller.GetPOV() == 270)
    arm.toggleCubeMode();
  if (controller.GetBButtonPressed())
      armSetPoint = 1000; //stowed
  else if (controller.GetXButtonPressed())
      armSetPoint = 4000; //human player
  else if (controller.GetYButtonPressed())
      armSetPoint = 3000; //high goal
  else if (controller.GetAButtonPressed())
      armSetPoint = 6000; //ground goal
  lastPosition = arm.getPosition();
  */
  //manual control overrides position control

  if (controller.GetLeftBumperReleased())
  {
    Pneumatic.shiftGears();
  }

  if (std::abs(controller.GetRightY()) > 0.3)
  {
    //manual control setpoint needed to not revert to the previous position once manual controls stop
    armSetPoint = 0;
    double armSpeed = (controller.GetRightY() - 0.3)/(0.7);
    arm.setSpeed(armSpeed);
  
  }
  else if (armSetPoint != 0)
  {
    //grab desired arm point on button press
    //this is sticky (we retain the setpoint until another input is received or manual control is initiated)
    if (controller.GetBButtonPressed())
      armSetPoint = 1000; //stowed
    else if (controller.GetXButtonPressed())
      armSetPoint = 4000; //human player
    else if (controller.GetYButtonPressed())
      armSetPoint = 3000; //high goal
    else if (controller.GetAButtonPressed())
      armSetPoint = 6000; //ground goal
    arm.setPosition(armSetPoint);
  }
  else
  {
    arm.setPosition(lastPosition);
    arm.setSpeed(0);
  }

  if (controller.GetBackButton())
  {
    drivetrain.turnToAngle(90);
  }

  //std::cout << "Arm position: " << arm.getPosition() << endl;
  //std::cout << "Drivetrain Left Velocity: " << drivetrain.getLeftVelocity() << endl;
  //std::cout << "Drivetrain Right Velocity: " << drivetrain.getRightVelocity() << endl;
  
}





void Robot::DisabledInit() {
  drivetrain.enableBrake(true);
  arm.resetEncoder();
  arm.brakeMode(true);
  arm.setSpeed(0);
  intake.setSpeed(0);
}





void Robot::DisabledPeriodic() 
{
  if (controller.GetAButtonPressed())
    arm.resetEncoder();
  std::cout << arm.getPosition() << endl;
  std::cout << gyro.GetPitch() << endl;
}





void Robot::TestInit() {
  arm.resetEncoder();
  arm.brakeMode(false);
  arm.setSpeed(0);
}





//use test to set manually move the arm and zero it
void Robot::TestPeriodic() {
  //Reset position once it's properly 0'd out
  if (controller.GetAButtonPressed())
    arm.resetEncoder();
  std::cout << arm.getPosition() << endl;
}





void Robot::SimulationInit() {}





void Robot::SimulationPeriodic() {}





#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
