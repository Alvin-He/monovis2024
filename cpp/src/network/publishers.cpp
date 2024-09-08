#pragma once 

#include "global.cpp"
#include "common.cpp"
#include "helpers.cpp"
#include "apriltag/apriltag.hpp"

#include <boost/asio/steady_timer.hpp>
#include <boost/algorithm/string.hpp>

#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"
#include "ntcore/networktables/DoubleTopic.h"
#include "network/redis.cpp"
#include <ranges>

#include <fmt/include/fmt/compile.h>

namespace Publishers
{
    namespace Internal {
        struct ApriltagPose {
            int id;
            Apriltag::World::RobotPose pose;
        };
        class ApriltagPosePublisher {
            private: 
            std::string pubUUID; 
            public:
            ApriltagPosePublisher(const std::string UUID) {
                this->pubUUID = UUID; 
            }

            cobalt::promise<void> operator()(std::vector<ApriltagPose> poses, int64_t timeStamp) {
                if (poses.size() <= 0) co_return; 

                // serialize vector<ApriltagPoses>
                std::ostringstream outputBuf; 
                for (auto& pose : poses) {
                    outputBuf << fmt::format(FMT_COMPILE("{}:{}:{}:{},"), pose.id, pose.pose.x, pose.pose.y, pose.pose.rot);
                }

                // generate redis key for this entry
                std::string key = fmt::format(FMT_COMPILE("Apriltag:{}:{}"), timeStamp, this->pubUUID);

                // construct and send the request
                redis::request req; 
                req.push("SET", key, outputBuf.str(), "PX", K::POSE_TIME_CONSIDERATION);
                co_await RedisDB::send(req); // detached send, don't care about result

                // for (auto &pose : poses) {
                //     auto tab = this->detectorTable->GetSubTable(std::to_string(pose.id));
                //     tab->PutNumber("x", pose.pose.x); 
                //     tab->PutNumber("y", pose.pose.y);
                //     tab->PutNumber("r", pose.pose.rot);
                //     tab->PutNumber("ts", timeStamp);
                // };
                co_return;
            };
        
            static cobalt::promise<std::tuple<std::vector<int64_t>, std::vector<ApriltagPose>>> receive() {
                std::vector<ApriltagPose> retPoseBuf; 
                std::vector<int64_t> retTimestamps;

                #ifdef FAIL_SILENT 
                try {
                #endif

                std::vector<std::string> matched = co_await RedisDB::scan("Apriltag:*");

                // deseralize all the timestamps and fill the req object for field values
                redis::request req;
                for (auto& key : matched) {
                    std::vector<std::string> keyCache(3);
                    boost::algorithm::split(keyCache, key, [](char c) {return c == ':';}); // split fields based on :
                    retTimestamps.push_back(std::stol(std::move(keyCache[1]))); 

                    req.push("GET", std::move(key));
                };

                // request field values
                auto responses = *co_await RedisDB::send(std::move(req)); 

                // deserialize all the pose info / values
                for (auto& res : responses) {
                    auto& value = res.value;
                    if (value.size() == 0) continue;

                    std::vector<std::string> poseStrings; 
                    boost::algorithm::split(poseStrings, value, [](char c) {return c == ',';}); // split csv based on ,

                    for (auto& poseString : poseStrings) {
                        if (poseString.size() < 7) continue; // invalid string, at least the delimeters and at least one data char per field should exist 
                        std::vector<std::string> fieldStrs(4);   
                        boost::algorithm::split(fieldStrs, poseString, [](char c) {return c == ':';}); // split fields based on :

                        // deserialize field strings into numbers 
                        retPoseBuf.push_back(ApriltagPose {
                            .id = std::stoi(std::move(fieldStrs[0])),
                            .pose = Apriltag::World::RobotPose {
                                .x = std::stod(std::move(fieldStrs[1])),
                                .y = std::stod(std::move(fieldStrs[2])),
                                .rot = std::stod(std::move(fieldStrs[3]))
                            }
                        });
                    };
                };
                
                #ifdef FAIL_SILENT
                } catch(...) {};
                #endif

                co_return std::tuple<std::vector<int64_t>, std::vector<ApriltagPose>>(std::move(retTimestamps), std::move(retPoseBuf));
            }
        };
    }

    class RobotPosePublisher {
        private:
        std::shared_ptr<nt::NetworkTable> roboPosTable; 
        nt::DoubleEntry xTopic, yTopic, rotTopic, tsTopic; 
        public:
        RobotPosePublisher(const std::string UUID, std::shared_ptr<nt::NetworkTable> root) :
            roboPosTable(root->GetSubTable("Vision")->GetSubTable("RobotPos")), 
            xTopic(std::move(roboPosTable->GetDoubleTopic("x").GetEntry(0))),
            yTopic(std::move(roboPosTable->GetDoubleTopic("y").GetEntry(0))),
            rotTopic(std::move(roboPosTable->GetDoubleTopic("r").GetEntry(0))),
            tsTopic(std::move(roboPosTable->GetDoubleTopic("ts").GetEntry(0)))
        {}
        cobalt::detached operator()(Apriltag::World::RobotPose pose, int64_t timestamp) {
            xTopic.Set(pose.x);
            yTopic.Set(pose.y);
            rotTopic.Set(pose.rot);
            tsTopic.Set(timestamp);
            co_return;
        }
    }; // RobotPosePublisher
} // namespace Publish
