#pragma once
#include "global.cpp"
#include "common.cpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <iterator>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "helpers.cpp"
#include "camera/cameraData.cpp"
#include <fmt/include/fmt/core.h> // why tf is fmt not defined here even when damn common.cpp exist

// and it was fine yesterday
namespace Apriltag {

struct EstimationResult {
    std::shared_ptr<Camera::CameraData> cameraInfo; 
    int id; 
    cv::Mat1d camToTagRvec; 
    cv::Mat1d camToTagTvec; 
};
typedef std::vector<EstimationResult> AllEstimationResults; 

class Estimator {
    public:
        Estimator(std::shared_ptr<Camera::CameraData> cameraData, cv::aruco::DetectorParameters detectorParams): 
            m_cameraData(cameraData), 
            m_detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), detectorParams)
        {
        }; // Estimator

        // detect an apriltag in an image
        // this method can be used in threads with out locking Estimator 
        AllEstimationResults Detect(cv::Mat image) {

            // detecting the tags in image
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids; 
            // cv::Mat gray; 
            // cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            this->m_detector.detectMarkers(image, corners, ids); 
            
            #if defined(DEBUG) && defined(GUI)
            cv::aruco::drawDetectedMarkers(image, corners, ids); 
            cv::imshow(fmt::format("camera-{} markers", m_cameraData->id), image); 
            #endif

            // solvepnp to generate cords
            int maxI = ids.size();
            AllEstimationResults estimations; 
            estimations.reserve(maxI);
            for (int i = 0; i < maxI; i++) {
                cv::Mat1d rvec;
                cv::Mat1d tvec; 
                cv::solvePnP(
                    m_objectPoints, 
                    corners[i], 
                    m_cameraData->matrix, 
                    m_cameraData->distCoeffs,
                    rvec, 
                    tvec, 
                    false, 
                    cv::SOLVEPNP_SQPNP); 
                // if (!ret) continue; // solve pnp "should" always return somehting

                // math
                cv::Mat1d rmat; 
                cv::Rodrigues(rvec, rmat); 
                cv::Mat1d theta = h::rad2deg(h::rodRotMatToEuler(rmat));

                cv::Mat1d rmatT; 
                cv::transpose(rmat,rmatT);
                cv::Mat1d pmat = (rmatT * tvec) * -1; // element wise mutiplication of negated rmat row 0
                pmat *= APRILTAG_BLOCK_SIZE_cm;
                
                // pamt and theta is already flat for index access

                estimations.emplace_back(m_cameraData, ids[i], theta, pmat); 
            }
            estimations.shrink_to_fit();
            return std::move(estimations); 
        }; // Detect

        cobalt::promise<AllEstimationResults> PromiseDetect(cv::Mat image) {
            co_return std::move(Detect(image)); 
        }
        cobalt::task<AllEstimationResults> TaskDetect(cv::Mat image) {
            co_return std::move(Detect(image)); 
        }

    private:
        std::shared_ptr<Camera::CameraData> m_cameraData; 
        cv::aruco::ArucoDetector m_detector;
        std::array<cv::Point3f, 4> m_objectPoints = {
            cv::Point3f {-4, 4, 0},
            cv::Point3f { 4, 4, 0},
            cv::Point3f { 4,-4, 0},
            cv::Point3f {-4,-4, 0}
        };
};
}// namespace Apriltag