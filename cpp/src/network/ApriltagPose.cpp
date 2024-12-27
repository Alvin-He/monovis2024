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
#include "worldPose/TypeDefs.cpp"
#include "camera/CameraData.cpp"
#include <fmt/include/fmt/compile.h>
#include <map>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include "apriltag/Estimator.ipp"

namespace Network {
namespace ApriltagPose {
    static std::string kNTVisionTable = "Vision";
    static std::string KNTCamerasTable = "Vision/Cameras";

    class Publisher { // interface
        public:
        Publisher() = default;
        virtual ~Publisher() = default; 

        virtual cobalt::promise<void> publish(std::vector<Apriltag::EstimationResult> estimations, int64_t timeStamp) = 0;
    }; // Publisher interface

    class Receiver { // interface
        public: 
        Receiver() = default;
        virtual ~Receiver() = default; 

        virtual cobalt::promise<std::tuple<std::vector<int64_t>, std::vector<Apriltag::EstimationResult>>> receive() = 0;
    }; // Receiver interface

    class NTPublisher : public Publisher {
        public:
        NTPublisher(const std::string cameraUUID) :
            m_UUID(cameraUUID)
        {
            auto cameras = nt::NetworkTableInstance::GetDefault().GetTable(KNTCamerasTable); 
            m_tThisCamera = cameras->GetSubTable(m_UUID);
        }

        cobalt::promise<void> publish(std::vector<Apriltag::EstimationResult> estimations, int64_t timeStamp) {
            if (estimations.size() <= 0) co_return; 

            for (auto &pose : estimations) {
                // cache look up to improve performance
                auto tabPos = m_tagIDTableCache.find(pose.id);

                std::shared_ptr<nt::NetworkTable> tab;
                if (tabPos == m_tagIDTableCache.end()) {
                    tab = m_tThisCamera->GetSubTable(std::to_string(pose.id));
                    m_tagIDTableCache.emplace(pose.id, tab); 
                } else {
                    tab = tabPos->second; 
                }
                
                // opencv mat serialization 
                std::array<double, 3> TvecData {
                    pose.camToTagTvec(0), pose.camToTagTvec(1), pose.camToTagTvec(2)
                };  

                std::array<double, 3> RvecData {
                    pose.camToTagRvec(0), pose.camToTagRvec(1), pose.camToTagRvec(2)
                };

                // data upload
                tab->PutString("camId", m_UUID);
                tab->PutNumber("tagId", pose.id);
                tab->PutNumberArray("tvec", TvecData);
                tab->PutNumberArray("rvec", RvecData);
                tab->PutNumber("ts", timeStamp);
                tab->PutBoolean("isUsed", false);
            };
            co_return;
        };
        private: 
            const std::string m_UUID;

            std::shared_ptr<nt::NetworkTable> m_tThisCamera = nullptr; 
            std::map<double, std::shared_ptr<nt::NetworkTable>> m_tagIDTableCache = {}; 
    }; // NTPublisher


    class NTReceiver : public Receiver {
        public:
        NTReceiver() {
            this->m_table = nt::NetworkTableInstance::GetDefault().GetTable(KNTCamerasTable);

            m_table->AddSubTableListener([&] (nt::NetworkTable *, std::basic_string_view<char>, std::shared_ptr<nt::NetworkTable> cameraTable){
                cameraTable->AddSubTableListener([&] (nt::NetworkTable *, std::basic_string_view<char>, std::shared_ptr<nt::NetworkTable> tagTable)  {
                    m_estimationResultTables.push_back(tagTable);
                });
            }); 

            fmt::println("WARN: Publishers::NT::ApriltagPosePublisher for NetworkTables is being used. This is not compatiable with mutiple cameras, running another publisher to the same networktables will result in data lost if the same tag is detected in more than 1 camera!");
        }
        
        cobalt::promise<std::tuple<std::vector<int64_t>, std::vector<Apriltag::EstimationResult>>> receive() {
            std::vector<int64_t> retTimestamps; 
            std::vector<Apriltag::EstimationResult> retResults;             

            for (auto& tagTab : m_estimationResultTables) {
                bool used = tagTab->GetBoolean("isUsed", false); 
                if (used) continue;
                tagTab->PutBoolean("isUsed", true); 

                auto cameraUUID = tagTab->GetString("camID", "null");
                if (cameraUUID == "null") continue;
                
                auto rawTvec = tagTab->GetNumberArray("tvec", {});
                if (rawTvec.size() != 3) continue;
                cv::Mat1d tvec {rawTvec[0], rawTvec[1], rawTvec[2]}; 

                auto rawRvec = tagTab->GetNumberArray("rvec", {}); 
                if (rawRvec.size() != 3) continue;
                cv::Mat1d rvec {rawRvec[0], rawRvec[1], rawRvec[2]}; 

                auto camDataPos = Camera::GlobalCameraRegistra.find(cameraUUID); 
                if (camDataPos == Camera::GlobalCameraRegistra.end()) continue;
                auto camData = camDataPos->second; 


                Apriltag::EstimationResult res {
                    .cameraInfo = camData,
                    .id = camData->id, 
                    .camToTagRvec = rvec,
                    .camToTagTvec = tvec
                };

                int64_t ts = tagTab->GetNumber("ts", 0);

                retResults.emplace_back(std::move(res));
                retTimestamps.push_back(ts); 
            }

            co_return {std::move(retTimestamps), std::move(retResults)};
        };

        private: 
            std::shared_ptr<nt::NetworkTable> m_table; 
            std::vector<std::shared_ptr<nt::NetworkTable>> m_estimationResultTables;  
    }; // NTReceiver

    /** Redis module needs a redo to be compatiable with the new scheme, low priority tho
    class RedisPublisher : public Publisher {
        public:
        RedisPublisher(const std::string UUID) {
            this->m_pubUUID = UUID; 
        }

        cobalt::promise<void> publish(std::vector<WorldPose::Pos2DwTag> poses, int64_t timeStamp) {
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

    class RedisReceiver : public Receiver {
        private: 
        std::string m_pubUUID; 
        public:
        RedisReceiver(const std::string UUID) {
            this->m_pubUUID = UUID; 
        }

        cobalt::promise<std::tuple<std::vector<int64_t>, std::vector<WorldPose::Pos2DwTag>>> receive() {
            std::vector<WorldPose::Pos2DwTag> retPoseBuf; 
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
                    retPoseBuf.push_back(WorldPose::Pos2DwTag {
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

            co_return std::tuple<std::vector<int64_t>, std::vector<WorldPose::Pos2DwTag>>(std::move(retTimestamps), std::move(retPoseBuf));
        }  
    }; // RedisReceiver
    */
};}
