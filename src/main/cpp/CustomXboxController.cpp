// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CustomXboxController.h"

#include <hal/FRCUsageReporting.h>

#include "frc/event/BooleanEvent.h"

using namespace frc;

CustomXboxController::CustomXboxController(int port) : GenericHID(port) {
  HAL_Report(HALUsageReporting::kResourceType_XboxController, port + 1);
}

double CustomXboxController::GetLeftX() const {
  return GetRawAxis(Axis::kLeftX);
}

double CustomXboxController::GetRightX() const {
  return GetRawAxis(Axis::kRightX);
}

double CustomXboxController::GetLeftY() const {
  return GetRawAxis(Axis::kLeftY);
}

double CustomXboxController::GetRightY() const {
  return GetRawAxis(Axis::kRightY);
}

double CustomXboxController::GetLeftTriggerAxis() const {
  return GetRawAxis(Axis::kLeftTrigger);
}

double CustomXboxController::GetRightTriggerAxis() const {
  return GetRawAxis(Axis::kRightTrigger);
}

int CustomXboxController::GetDPAD() const{
  return GetPOV(0);
}

bool CustomXboxController::GetLeftBumper() const {
  return GetRawButton(Button::kLeftBumper);
}

bool CustomXboxController::GetRightBumper() const {
  return GetRawButton(Button::kRightBumper);
}

bool CustomXboxController::GetLeftBumperPressed() {
  return GetRawButtonPressed(Button::kLeftBumper);
}

bool CustomXboxController::GetRightBumperPressed() {
  return GetRawButtonPressed(Button::kRightBumper);
}

bool CustomXboxController::GetLeftBumperReleased() {
  return GetRawButtonReleased(Button::kLeftBumper);
}

bool CustomXboxController::GetRightBumperReleased() {
  return GetRawButtonReleased(Button::kRightBumper);
}

BooleanEvent CustomXboxController::LeftBumper(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetLeftBumper(); });
}

BooleanEvent CustomXboxController::RightBumper(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetRightBumper(); });
}

bool CustomXboxController::GetLeftStickButton() const {
  return GetRawButton(Button::kLeftStick);
}

bool CustomXboxController::GetRightStickButton() const {
  return GetRawButton(Button::kRightStick);
}

bool CustomXboxController::GetLeftStickButtonPressed() {
  return GetRawButtonPressed(Button::kLeftStick);
}

bool CustomXboxController::GetRightStickButtonPressed() {
  return GetRawButtonPressed(Button::kRightStick);
}

bool CustomXboxController::GetLeftStickButtonReleased() {
  return GetRawButtonReleased(Button::kLeftStick);
}

bool CustomXboxController::GetRightStickButtonReleased() {
  return GetRawButtonReleased(Button::kRightStick);
}

BooleanEvent CustomXboxController::LeftStick(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetLeftStickButton(); });
}

BooleanEvent CustomXboxController::RightStick(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetRightStickButton(); });
}

bool CustomXboxController::GetAButton() const {
  return GetRawButton(Button::kA);
}

bool CustomXboxController::GetAButtonPressed() {
  return GetRawButtonPressed(Button::kA);
}

bool CustomXboxController::GetAButtonReleased() {
  return GetRawButtonReleased(Button::kA);
}

BooleanEvent CustomXboxController::A(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetAButton(); });
}

bool CustomXboxController::GetBButton() const {
  return GetRawButton(Button::kB);
}

bool CustomXboxController::GetBButtonPressed() {
  return GetRawButtonPressed(Button::kB);
}

bool CustomXboxController::GetBButtonReleased() {
  return GetRawButtonReleased(Button::kB);
}

BooleanEvent CustomXboxController::B(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetBButton(); });
}

bool CustomXboxController::GetXButton() const {
  return GetRawButton(Button::kX);
}

bool CustomXboxController::GetXButtonPressed() {
  return GetRawButtonPressed(Button::kX);
}

bool CustomXboxController::GetXButtonReleased() {
  return GetRawButtonReleased(Button::kX);
}

BooleanEvent CustomXboxController::X(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetXButton(); });
}

bool CustomXboxController::GetYButton() const {
  return GetRawButton(Button::kY);
}

bool CustomXboxController::GetYButtonPressed() {
  return GetRawButtonPressed(Button::kY);
}

bool CustomXboxController::GetYButtonReleased() {
  return GetRawButtonReleased(Button::kY);
}

BooleanEvent CustomXboxController::Y(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetYButton(); });
}

bool CustomXboxController::GetBackButton() const {
  return GetRawButton(Button::kBack);
}

bool CustomXboxController::GetBackButtonPressed() {
  return GetRawButtonPressed(Button::kBack);
}

bool CustomXboxController::GetBackButtonReleased() {
  return GetRawButtonReleased(Button::kBack);
}

BooleanEvent CustomXboxController::Back(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetBackButton(); });
}

bool CustomXboxController::GetStartButton() const {
  return GetRawButton(Button::kStart);
}

bool CustomXboxController::GetStartButtonPressed() {
  return GetRawButtonPressed(Button::kStart);
}

bool CustomXboxController::GetStartButtonReleased() {
  return GetRawButtonReleased(Button::kStart);
}

BooleanEvent CustomXboxController::Start(EventLoop* loop) const {
  return BooleanEvent(loop, [this]() { return this->GetStartButton(); });
}

BooleanEvent CustomXboxController::LeftTrigger(double threshold,
                                         EventLoop* loop) const {
  return BooleanEvent(loop, [this, threshold]() {
    return this->GetLeftTriggerAxis() > threshold;
  });
}

BooleanEvent CustomXboxController::LeftTrigger(EventLoop* loop) const {
  return this->LeftTrigger(0.5, loop);
}

BooleanEvent CustomXboxController::RightTrigger(double threshold,
                                          EventLoop* loop) const {
  return BooleanEvent(loop, [this, threshold]() {
    return this->GetRightTriggerAxis() > threshold;
  });
}

BooleanEvent CustomXboxController::RightTrigger(EventLoop* loop) const {
  return this->RightTrigger(0.5, loop);
}
