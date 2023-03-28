#include "Intake.h"

Intake::Intake()
{
    mLeftIntake.RestoreFactoryDefaults();
    mRightIntake.RestoreFactoryDefaults();

    mRightIntake.SetInverted(true);

    mRightIntake.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    mLeftIntake.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Intake::setSpeed(double speed)
{
    mLeftIntake.Set(speed);
    mRightIntake.Set(speed);
}
