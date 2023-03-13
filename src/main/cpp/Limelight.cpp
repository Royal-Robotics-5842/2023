#include "Limelight.h"

using namespace std;

Limelight::Limelight()
{
}

void Limelight::updatePose()
{
    
    double spanArray[6] = {0,0,0,0,0,0};
    std::span<double> defaultSpan{spanArray};
    std::vector<double> botPoseData = table->GetNumberArray("botpose",defaultSpan);
    if (botPoseData.empty())
        botPoseData = {0,0,0,0,0,0};
    frc::Translation3d translation{botPoseData[0]*1_m, botPoseData[1]*1_m, botPoseData[2]*1_m};
    frc::Rotation3d rotation{botPoseData[3]*1_rad, botPoseData[4]*1_rad, botPoseData[5]*1_rad};
    frc::Pose3d limelightPose{translation, rotation};

    double x = limelightPose.X()/1_m;
    double y = limelightPose.Y()/1_m;

    cout<<"("<<x<<", "<<y<<")"<<endl;
};