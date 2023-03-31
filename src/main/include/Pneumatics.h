#pragma once

#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <units/current.h>
#include "Constants.h"


class Pneumatics
{
    frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
    frc::Solenoid   SolenoidPCM{frc::PneumaticsModuleType::CTREPCM, 0};

    bool enabled = pcmCompressor.Enabled();
    bool pressureSwitch = pcmCompressor.GetPressureSwitchValue();
    double current = pcmCompressor.GetCurrent()/1_A;

public:
    Pneumatics();
    void shiftGears();
};