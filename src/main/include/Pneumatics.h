
#pragma once

#include "frc/PneumaticHub.h"
#include "frc/Solenoid.h"
#include <units/current.h>
#include "Constants.h"


class Pneumatics
{
    frc::PneumaticHub mHub{1};
    frc::Solenoid mShifter{frc::PneumaticsModuleType::REVPH, 15};

    bool enabled;
    bool pressureSwitch;
    double current;

public:
    Pneumatics();
    void shiftGears();
    void enableCompressor();
};