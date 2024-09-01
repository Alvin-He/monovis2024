#pragma once 

#include "global.cpp"
#include "common.cpp"
#include "helpers.cpp"
#include "apriltag/apriltag.hpp"

#include <boost/asio/steady_timer.hpp>

#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"
#include "ntcore/networktables/DoubleTopic.h"

namespace Publishers
{
    namespace Internal {
        struct ApriltagPose {
            int id;
            Apriltag::World::RobotPose pose;
        };
        class ApriltagPosePublisher {
            private: 
            std::shared_ptr<nt::NetworkTable> detectorTable; 
            public:
            ApriltagPosePublisher(const std::string UUID, std::shared_ptr<nt::NetworkTable> root) {
                this->detectorTable = root->GetSubTable("Apriltag")->GetSubTable(UUID); 
            }
            cobalt::detached operator()(std::vector<ApriltagPose> poses, int64_t timeStamp) {
                for (auto &pose : poses) {
                    auto tab = this->detectorTable->GetSubTable(std::to_string(pose.id));
                    tab->PutNumber("x", pose.pose.x); 
                    tab->PutNumber("y", pose.pose.y);
                    tab->PutNumber("r", pose.pose.rot);
                    tab->PutNumber("ts", timeStamp);
                };
                co_return;
            };
        };
    }

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
