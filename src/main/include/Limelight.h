#pragma once

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/SpanExtras.h"
#include <vector>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

class limelightClass {
 public:
        limelightClass();
        void getPose();
};