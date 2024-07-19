#pragma once 

#include "global.cpp"
#include "helpers.cpp"
#include "apriltag/apriltag.hpp"


#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"
#include "ntcore/networktables/DoubleTopic.h"

namespace Publishers
{

    struct RobotPosePacket {
        Apriltag::World::RobotPose pose; 
        int64_t timestamp;
    };
    cobalt::generator<bool, RobotPosePacket> RobotPosePublisher(std::shared_ptr<nt::NetworkTable> poseTable) {
        // not me using generators as a class lmao
        nt::DoubleEntry xTopic = poseTable->GetDoubleTopic("x").GetEntry(0);
        nt::DoubleEntry yTopic = poseTable->GetDoubleTopic("y").GetEntry(0); 
        nt::DoubleEntry rotTopic = poseTable->GetDoubleTopic("r").GetEntry(0); 
        nt::DoubleEntry tsTopic = poseTable->GetDoubleTopic("ts").GetEntry(0); 

        for(;;) {
            RobotPosePacket packet = co_yield true; 
            xTopic.Set(packet.pose.x); 
            yTopic.Set(packet.pose.y); 
            rotTopic.Set(packet.pose.rot); 
            tsTopic.Set(packet.timestamp); 
        }
    } // RobotPosePublisher
} // namespace Publish
