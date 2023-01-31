// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Drivetrain.h"
#include "frc/XboxController.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain drivetrain;
frc::XboxController controller{1};

void Robot::RobotInit() 
{

}

void Robot::RobotPeriodic() 
{
  drivetrain.updatePose();
}

void Robot::AutonomousInit() 
{



}

void Robot::AutonomousPeriodic() 
{

}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() 
{

  drivetrain.drive(controller.GetLeftY(), controller.GetRightY());

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif