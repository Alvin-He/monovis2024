#pragma once

#include "fmt/core.h"
#include "global.cpp"
#include "common.cpp"

#include <array>
#include <boost/asio/steady_timer.hpp>
#include <boost/algorithm/string.hpp>

#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"
#include "ntcore/networktables/DoubleTopic.h"
#include "redis.cpp"

#include <fmt/include/fmt/compile.h>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace Network {
namespace ApriltagPose {
    static std::string kNTApriltagTableLocation = "SmartDashboard/Apriltags";

    class Publisher { // interface
        public:
        Publisher() = default;
        virtual ~Publisher() = default; 

        virtual cobalt::promise<void> publish(std::vector<Apriltag::World::Pos2DwTag> poses, int64_t timeStamp) = 0;
    }; // Publisher interface

    class NTPublisher : public Publisher {
        public:
        NTPublisher() {
            fmt::println("WARN: Publishers::NT::ApriltagPosePublisher for NetworkTables is being used. This is not compatiable with mutiple cameras, running another publisher to the same networktables will result in data lost if the same tag is detected in more than 1 camera!");
            this->m_table = nt::NetworkTableInstance::GetDefault().GetTable(kNTApriltagTableLocation); 
        }

        cobalt::promise<void> publish(std::vector<Apriltag::World::Pos2DwTag> poses, int64_t timeStamp) {
            if (poses.size() <= 0) co_return; 

            for (auto &pose : poses) {
                auto tab = m_table->GetSubTable(std::to_string(pose.id));
                tab->PutNumber("x", pose.x); 
                tab->PutNumber("y", pose.y);
                tab->PutNumber("r", pose.rot);
                tab->PutNumber("ts", timeStamp);
                tab->PutBoolean("isUsed", false);
            };
            co_return;
        };
        private: 
            std::shared_ptr<nt::NetworkTable> m_table; 
    }; // NTPublisher

    class RedisPublisher : public Publisher {
        public:
        RedisPublisher(const std::string UUID) {
            this->m_pubUUID = UUID; 
        }

        cobalt::promise<void> publish(std::vector<Apriltag::World::Pos2DwTag> poses, int64_t timeStamp) {
            if (poses.size() <= 0) co_return; 

            // serialize vector<ApriltagPoses>
            std::ostringstream outputBuf; 
            for (auto& pose : poses) {
                outputBuf << fmt::format(FMT_COMPILE("{}:{}:{}:{},"), pose.id, pose.x, pose.y, pose.rot);
            }

            // generate redis key for this entry
            std::string key = fmt::format(FMT_COMPILE("Apriltag:{}:{}"), timeStamp, this->m_pubUUID);

            // construct and send the request
            redis::request req; 
            req.push("SET", key, outputBuf.str(), "PX", K::POSE_TIME_CONSIDERATION);
            co_await RedisDB::noQueueSendDiscard(req); // detached send, don't care about result
            co_return;
        };

        private: 
            std::string m_pubUUID; 
    }; // RedisPublisher

    class Receiver { // interface
        public: 
        Receiver() = default;
        virtual ~Receiver() = default; 

        virtual cobalt::promise<std::tuple<std::vector<int64_t>, std::vector<Apriltag::World::Pos2DwTag>>> receive() = 0;
    }; // Receiver interface

    class NTReceiver : public Receiver {
        public:
        NTReceiver() {
            this->m_table = nt::NetworkTableInstance::GetDefault().GetTable(kNTApriltagTableLocation); 
            fmt::println("WARN: Publishers::NT::ApriltagPosePublisher for NetworkTables is being used. This is not compatiable with mutiple cameras, running another publisher to the same networktables will result in data lost if the same tag is detected in more than 1 camera!");
            m_table->AddSubTableListener([&](nt::NetworkTable* tab, std::string_view name, std::shared_ptr<nt::NetworkTable> table) {
                m_subtables.push_back(std::move(name)); 
            });
        }

        cobalt::promise<std::tuple<std::vector<int64_t>, std::vector<Apriltag::World::Pos2DwTag>>> receive() {
            std::vector<int64_t> retTimestamps; 
            std::vector<Apriltag::World::Pos2DwTag> retPoseBuf;             

            // auto subtables = m_table->GetSubTables(); 
            for (auto&& tagIdPath : m_subtables) {
                auto tagTab = m_table->GetSubTable(tagIdPath);

                bool used = tagTab->GetBoolean("isUsed", false); 
                if (used) continue;
                tagTab->PutBoolean("isUsed", true); 

                Apriltag::World::Pos2DwTag res;
                res.x = tagTab->GetNumber("x", 0);
                res.y = tagTab->GetNumber("y", 0);
                res.rot = tagTab->GetNumber("r", 0);
                res.id = std::stoi(tagIdPath.data()); 
                int64_t ts = tagTab->GetNumber("ts", 0);

                retPoseBuf.push_back(std::move(res));
                retTimestamps.push_back(ts); 
            }

            co_return std::tuple<std::vector<int64_t>, std::vector<Apriltag::World::Pos2DwTag>>(std::move(retTimestamps), std::move(retPoseBuf));

        };
        private: 
            std::shared_ptr<nt::NetworkTable> m_table; 
            std::vector<std::string_view> m_subtables; 
    }; // NTReceiver

    class RedisReceiver : public Receiver {
        private: 
        std::string m_pubUUID; 
        public:
        RedisReceiver(const std::string UUID) {
            this->m_pubUUID = UUID; 
        }

        cobalt::promise<std::tuple<std::vector<int64_t>, std::vector<Apriltag::World::Pos2DwTag>>> receive() {
            std::vector<Apriltag::World::Pos2DwTag> retPoseBuf; 
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
            auto responses = *co_await RedisDB::send(req); 

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
                    retPoseBuf.push_back(Apriltag::World::Pos2DwTag {
                        .x = std::stod(std::move(fieldStrs[1])),
                        .y = std::stod(std::move(fieldStrs[2])),
                        .rot = std::stod(std::move(fieldStrs[3])),
                        .id = std::stoi(std::move(fieldStrs[0]))
                    });
                };
            };
            
            #ifdef FAIL_SILENT
            } catch(...) {};
            #endif

            co_return std::tuple<std::vector<int64_t>, std::vector<Apriltag::World::Pos2DwTag>>(std::move(retTimestamps), std::move(retPoseBuf));
        }  
    }; // RedisReceiver

};}
