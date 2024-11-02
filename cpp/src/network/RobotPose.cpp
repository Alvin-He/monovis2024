#pragma once

#include "cobalt/detached.hpp"
#include "fmt/core.h"
#include "global.cpp"
#include "common.cpp"

#include <boost/asio/steady_timer.hpp>
#include <boost/algorithm/string.hpp>

#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"
#include "ntcore/networktables/DoubleTopic.h"

#include <cstdint>
#include <fmt/include/fmt/compile.h>
#include <ntcore_cpp.h>

namespace Network {
namespace RobotPose {
    class Publisher { // interface
        public: 
        Publisher() = default;
        virtual ~Publisher() = default; 
        
        virtual cobalt::detached publish(Apriltag::World::Pos2D pose, int64_t timestamp) = 0;
    }; // Publisher interface


    class NTPublisher : public Publisher {
        public:
        NTPublisher(const std::string UUID) :
            roboPosTable(nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetSubTable("Vision")->GetSubTable("RobotPos")), 
            xTopic(std::move(roboPosTable->GetDoubleTopic("x").GetEntry(0))),
            yTopic(std::move(roboPosTable->GetDoubleTopic("y").GetEntry(0))),
            rotTopic(std::move(roboPosTable->GetDoubleTopic("r").GetEntry(0))),
            tsTopic(std::move(roboPosTable->GetDoubleTopic("ts").GetEntry(0)))
        {}
        cobalt::detached publish(Apriltag::World::Pos2D pose, int64_t timestamp) {
            xTopic.Set(pose.x);
            yTopic.Set(pose.y);
            rotTopic.Set(pose.rot);
            tsTopic.Set(timestamp);
            co_return;
        }
        private:
            std::shared_ptr<nt::NetworkTable> roboPosTable; 
            nt::DoubleEntry xTopic, yTopic, rotTopic, tsTopic; 
    }; // NTPublisher
}
};