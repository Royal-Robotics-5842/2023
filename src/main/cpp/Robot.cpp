// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Drivetrain.h"
#include "Arm.h"
#include "Intake.h"
#include "frc/XboxController.h"

#include <fmt/core.h>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain drivetrain;
frc::XboxController controller{1};
Arm arm;
Intake intake;
frc::Timer t;

int auton;

void Robot::RobotInit() 
{
    arm.resetPosition();

}





void Robot::RobotPeriodic() 
{
  drivetrain.updatePose();

  std::cout << "Arm position: " << arm.getPosition() << endl;
  std::cout << "Gyro Yaw: " << drivetrain.getYaw() << endl;
}





void Robot::AutonomousInit() 
{
  switch (controller.GetPOV())
  {
    case (0):  
      auton = 1;
      break;
  
   case (90):
      auton = 2;
      break;
  
    case (180):
      auton = 3;
      break;

    case (270):
      auton = 4;
      break;
  
    default:
      auton = 1;
      break;
  }
  t.Restart();
}





void Robot::AutonomousPeriodic() 
{
  units::second_t autoTime = t.Get();
  switch (auton)
  {
   // case (1):
   // if ()

    
  case (1):
    if (autoTime < 3_s) {
	    arm.setPosition(3);
      intake.setSpeed(.3);}
    else if (autoTime < 4_s)
  	  intake.setSpeed(-.3); 
    else if (autoTime < 7_s)
  	  arm.setPosition(0);
    else if (autoTime < 8_s)
      arm.brakeMode(true);
    else if (autoTime < 11_s)
      drivetrain.drive(-.5, -.5);
    else if (autoTime < 15_s)
      drivetrain.turnToAngle(180);
    break;
  case (3):
    if (autoTime < 2_s)
	    arm.setPosition(3);
    else if (autoTime < 3_s)
  	  intake.setSpeed(-.3); 
    else if (autoTime < 5_s)
  	  arm.setPosition(0);
    else if (autoTime < 6_s)
      arm.brakeMode(true);
    else if (autoTime < 10_s)
      drivetrain.drive(.5, .5);
    else if (autoTime  <= 15_s)
      drivetrain.autobalance();
    break;
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

  if(controller.GetPOV() == 180) {
     drivetrain.autobalance();
  }

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
  if (controller.GetRightTriggerAxis() > 0.1)
    intake.setSpeed(.3 * sign);
  else if (controller.GetLeftTriggerAxis() > 0.1)
    intake.setSpeed(-.3 * sign);
  else
    intake.setSpeed(0);

  //toggle between cone preset heights and cube preset heights -- default is cube
  //if limelight, also toggle vision mode when aiming between tape and tags
  if (controller.GetLeftBumperPressed())
    arm.toggleMode();

  if (controller.GetAButtonPressed())
    armSetPoint = 1000;
  else if (controller.GetXButtonPressed())
    armSetPoint = 2000;
  else if (controller.GetYButtonPressed())
    armSetPoint = 3000;
  else if (controller.GetBButtonPressed())
    armSetPoint = 4000;

  lastPosition = arm.getPosition();
  //manual control overrides position control
  if (std::abs(controller.GetRightY()) > 0.3)
  {
    //manual control setpoint needed to not revert to the previous position once manual controls stop
    armSetPoint = 0;
    double armSpeed = (controller.GetRightY() - 0.3)/(0.7);
    arm.setSpeed(-1*armSpeed);
  }
  else if (armSetPoint != 0)
  {
    //grab desired arm point on button press
    //this is sticky (we retain the setpoint until another input is received or manual control is initiated)
    if (controller.GetBButtonPressed())
      armSetPoint = 4000; //stowed
    else if (controller.GetXButtonPressed())
      armSetPoint = 2000; //mid goal
    else if (controller.GetYButtonPressed())
      armSetPoint = 3000; //high goal
    else if (controller.GetAButtonPressed())
      armSetPoint = 1000; //low goal
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
  arm.brakeMode(true);
}





void Robot::DisabledPeriodic() {}





void Robot::TestInit() {
  arm.resetPosition();
  arm.brakeMode(false);
  arm.setSpeed(0);
}





//use test to set manually move the arm and zero it
void Robot::TestPeriodic() {
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
