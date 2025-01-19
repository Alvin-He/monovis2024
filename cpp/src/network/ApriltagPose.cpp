#pragma once

#include "fmt/core.h"
#include "global.cpp"
#include "common.cpp"

#include <algorithm>
#include <array>
#include <boost/asio/steady_timer.hpp>
#include <boost/algorithm/string.hpp>

#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"
#include "ntcore/networktables/DoubleTopic.h"
#include "ntcore/networktables/IntegerArrayTopic.h"
#include "ntcore/networktables/DoubleArrayTopic.h"
#include "ntcore/networktables/IntegerTopic.h"
#include "redis.cpp"
#include "world/TypeDefs.cpp"
#include "camera/CameraData.cpp"
#include <cstdint>
#include <fmt/include/fmt/compile.h>
#include <map>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <ranges>
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

    // base interface for publishing an EstimationResult onto Network 
    class Publisher { // interface
        public:
        Publisher() = default;
        virtual ~Publisher() = default; 

        virtual cobalt::promise<void> publish(const Apriltag::EstimationResult& estimations, const int64_t& timeStamp) = 0;
    }; // Publisher interface

    // base interface for receiving an EstimationResult from Network 
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
            m_idsPublisher = m_tThisCamera->GetIntegerArrayTopic("ids").Publish();
            m_cornersPublisher = m_tThisCamera->GetDoubleArrayTopic("corners").Publish();
            m_tsPublisher = m_tThisCamera->GetIntegerTopic("ts").Publish();
        }

        cobalt::promise<void> publish(const Apriltag::EstimationResult& estimations, const int64_t& timeStamp) {
            int count = estimations.ids.size();
            if (count <= 0) co_return; 

            // type cast to NT sendable types
            std::vector<int64_t> ids {estimations.ids.begin(), estimations.ids.end()};
            
            std::vector<double> corners; corners.reserve(count * 8);
            for (auto& tagCorners : estimations.corners) {
                auto pos = corners.end();
                corners.emplace(pos + 0, tagCorners[0].x);
                corners.emplace(pos + 1, tagCorners[0].y);
                corners.emplace(pos + 2, tagCorners[1].x);
                corners.emplace(pos + 3, tagCorners[1].y);
                corners.emplace(pos + 4, tagCorners[2].x);
                corners.emplace(pos + 5, tagCorners[2].y);
                corners.emplace(pos + 6, tagCorners[3].x);
                corners.emplace(pos + 7, tagCorners[3].y);

            }

            // data upload
            m_idsPublisher.Set(ids); 
            m_cornersPublisher.Set(corners); 
            m_tsPublisher.Set(timeStamp); 

            co_return;
        };
        private: 
            const std::string m_UUID;

            std::shared_ptr<nt::NetworkTable> m_tThisCamera = nullptr; 
            nt::IntegerArrayPublisher m_idsPublisher;
            nt::DoubleArrayPublisher m_cornersPublisher;
            nt::IntegerPublisher m_tsPublisher; 
            
    }; // NTPublisher


    class NTReceiver : public Receiver {
        public:
        NTReceiver() {
            this->m_table = nt::NetworkTableInstance::GetDefault().GetTable(KNTCamerasTable);

            m_table->AddSubTableListener([&] (nt::NetworkTable *, std::basic_string_view<char>camName_unowned, std::shared_ptr<nt::NetworkTable> cameraTable){
                if (camName_unowned.length() <= 0) return;
                std::string camName {camName_unowned}; // copy camName to memory owned by us

                m_cameras.emplace_back(CameraTableInfo {
                    .camera = cameraTable, 
                    .uuid = std::move(camName),
                    .idsSub = cameraTable->GetIntegerArrayTopic("ids").Subscribe({}), 
                    .cornersSub = cameraTable->GetDoubleArrayTopic("corners").Subscribe({}),
                    .tsSub = cameraTable->GetIntegerTopic("ts").Subscribe(0)
                });
            }); 
        }
        
        cobalt::promise<std::tuple<std::vector<int64_t>, std::vector<Apriltag::EstimationResult>>> receive() {
            std::vector<int64_t> retTimestamps; 
            std::vector<Apriltag::EstimationResult> retResults;             

            for (auto& camera : m_cameras) {
                auto ts = camera.tsSub.Get();
                if (ts == camera.lastTs) continue; // already processed this frame data
                camera.lastTs = ts;
                retTimestamps.push_back(std::move(ts)); 


                //type cast to processable types
                auto _ids = camera.idsSub.Get(); 
                std::vector<int> ids {_ids.begin(), _ids.end()};
                
                auto _corners = camera.cornersSub.Get(); auto _cornersCount = _corners.size();
                std::vector<std::vector<cv::Point2f>> corners;
                for (int i = 0; i < _cornersCount; i+=8) {
                    corners.emplace_back(std::vector<cv::Point2f> { // load all corners of a tag into vector as cv::Point2f
                        cv::Point2f(_corners[i + 0], _corners[i + 1]),
                        cv::Point2f(_corners[i + 2], _corners[i + 3]),
                        cv::Point2f(_corners[i + 4], _corners[i + 5]), 
                        cv::Point2f(_corners[i + 6], _corners[i + 7]), 
                    });
                }


                retResults.emplace_back(Apriltag::EstimationResult {
                    .cameraInfo = Camera::GlobalCameraRegistra[camera.uuid], 
                    .ids = std::move(ids), 
                    .corners = std::move(corners)
                });
            }

            co_return {std::move(retTimestamps), std::move(retResults)};
        };

        private: 
            struct CameraTableInfo {
                std::shared_ptr<nt::NetworkTable> camera;
                std::string uuid;
                nt::IntegerArraySubscriber idsSub;
                nt::DoubleArraySubscriber cornersSub;
                nt::IntegerSubscriber tsSub; 
                int64_t lastTs = 0;
            };

            std::shared_ptr<nt::NetworkTable> m_table; 
            std::vector<CameraTableInfo> m_cameras;
    }; // NTReceiver

    /** Redis module needs a redo to be compatiable with the new scheme, low priority tho
    class RedisPublisher : public Publisher {
        public:
        RedisPublisher(const std::string UUID) {
            this->m_pubUUID = UUID; 
        }

        cobalt::promise<void> publish(std::vector<World::Pos2DwTag> poses, int64_t timeStamp) {
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

        cobalt::promise<std::tuple<std::vector<int64_t>, std::vector<World::Pos2DwTag>>> receive() {
            std::vector<World::Pos2DwTag> retPoseBuf; 
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
                    retPoseBuf.push_back(World::Pos2DwTag {
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

            co_return std::tuple<std::vector<int64_t>, std::vector<World::Pos2DwTag>>(std::move(retTimestamps), std::move(retPoseBuf));
        }  
    }; // RedisReceiver
    */
};}
