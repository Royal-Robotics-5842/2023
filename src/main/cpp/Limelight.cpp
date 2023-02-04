#include "limelight.h"
#include <iostream>
using namespace std;

std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");


limelightClass::limelightClass(){}

void limelightClass::getPose(){
    
    double spanarray[6] = {0,0,0,0,0,0};
    std::span<double> defaultReturn{spanarray};
    std::vector<double> botPoseData = table->GetNumberArray("botpose",defaultReturn);
    if (botPoseData.empty())
        botPoseData = {0,0,0,0,0,0};
     frc::Translation3d translation{botPoseData[0]*1_m, botPoseData[1]*1_m, botPoseData[2]*1_m};
     frc::Rotation3d rotation{botPoseData[3]*1_rad, botPoseData[4]*1_rad, botPoseData[5]*1_rad};
     frc::Pose3d limelightPose{translation, rotation};


    double tx = table->GetNumber("tx", 0.0);

    double x = limelightPose.X()/1_m;
    double y = limelightPose.Y()/1_m;

    cout<<"("<<x<<", "<<y<<")"<<endl;

    cout<<"tx: "<<tx<<endl;
};