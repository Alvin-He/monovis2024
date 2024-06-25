#pragma once 

#include "global.cpp"
#include "helpers.cpp"
#include "apriltag/apriltag.hpp"


#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"
#include "ntcore/networktables/DoubleTopic.h"

namespace Publishers
{
    cobalt::generator<bool, Apriltag::World::RobotPose> RobotPosePublisher(std::shared_ptr<nt::NetworkTable> poseTable) {
        // not me using generators as a class lmao
        nt::DoubleEntry xTopic = poseTable->GetDoubleTopic("x").GetEntry(0);
        nt::DoubleEntry yTopic = poseTable->GetDoubleTopic("y").GetEntry(0); 
        nt::DoubleEntry rotTopic = poseTable->GetDoubleTopic("r").GetEntry(0); 

        for(;;) {
            auto pose = co_yield true; 
            xTopic.Set(pose.x); 
            yTopic.Set(pose.y); 
            rotTopic.Set(pose.rot); 
        }
    } // RobotPosePublisher
} // namespace Publish
