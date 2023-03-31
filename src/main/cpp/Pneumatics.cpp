#pragma once
#include "Pneumatics.h"

Pneaumatics::Pneumatics()
{
    pcmCompressor.EnableDigital();
}

void Pneaumatics::shiftGears()
{
    SolenoidPCM.Toggle();
}