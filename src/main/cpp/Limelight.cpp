#include "Limelight.h"

Limelight::Limelight()
{
}

void Limelight::updatePose()
{
    double spanArray[] = {0, 0, 0, 0, 0, 0};
    std::span<double> defaultSpan(spanArray);
    std::vector<double> tableData = table->GetNumberArray("botpose", defaultSpan);
    if (tableData.empty())
        tableData = {0, 0, 0, 0, 0, 0};
    frc::Translation3d translation{tableData[0]*1_m, tableData[1]*1_m, tableData[2]*1_m};
    frc::Rotation3d rotation{tableData[3]*1_rad, tableData[4]*1_rad, tableData[5]*1_rad};
    frc::Pose3d pose = {translation, rotation};
}