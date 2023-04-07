
#include "Pneumatics.h"

Pneumatics::Pneumatics()
{
}

void Pneumatics::enableCompressor()
{
    mHub.EnableCompressorDigital();
}
void Pneumatics::shiftGears()
{
    mShifter.Toggle();
    //std::cout << SolenoidPCM.Toggle() << endl;
}
