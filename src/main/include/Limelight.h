#pragma once

#include "frc/smartdashboard/Smartdashboard.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

#include <vector>
#include <span>
#include <iostream>

class Limelight
{
    
public:
    Limelight();
    void updatePose();
};