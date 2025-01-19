#pragma once
#include "Estimator.ipp"
#include "helpers.cpp"
#include <array>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

// and it was fine yesterday
namespace Apriltag {

class OpenCVArucoEstimator : Estimator {
    public:
        OpenCVArucoEstimator (std::shared_ptr<Camera::CameraData> cameraData, cv::aruco::DetectorParameters detectorParams): 
            m_cameraData(cameraData), 
            m_detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), detectorParams)
        {
        }; // Estimator

        // detect an apriltag in an image
        // this method can be used in threads with out locking Estimator 
        EstimationResult Detect(cv::Mat image) override {
            // set up result obj, forwarding m_cameraData
            EstimationResult result {
                .cameraInfo = m_cameraData
            }; 

            // detecting the tags in image
            this->m_detector.detectMarkers(image, result.corners, result.ids); 
            
            // debug show image
            #if defined(DEBUG) && defined(GUI)
            cv::aruco::drawDetectedMarkers(image, result.corners, result.ids); 
            cv::imshow(fmt::format("camera-{} markers", m_cameraData->deviceID), image); 
            #endif

            // solvepnp to generate cords
            // int maxI = ids.size();
            // std::vector<EstimationResult> estimations; 
            // estimations.reserve(maxI);
            // for (int i = 0; i < maxI; i++) {
            //     cv::Mat1d rvec;
            //     cv::Mat1d tvec; 
            //     cv::solvePnP(
            //         m_objectPoints, 
            //         corners[i], 
            //         m_cameraData->matrix, 
            //         m_cameraData->distCoeffs,
            //         rvec, 
            //         tvec, 
            //         false, 
            //         cv::SOLVEPNP_SQPNP); 
            //     // if (!ret) continue; // solve pnp "should" always return somehting

            //     // math
            //     cv::Mat1d rmat; 
            //     cv::Rodrigues(rvec, rmat); 
            //     cv::Mat1d theta = h::rad2deg(h::rodRotMatToEuler(rmat));

            //     cv::Mat1d rmatT; 
            //     cv::transpose(rmat,rmatT);
            //     cv::Mat1d pmat = (rmatT * tvec) * -1; // element wise mutiplication of negated rmat row 0
            //     pmat *= APRILTAG_BLOCK_SIZE_cm;
                
            //     // pamt and theta is already flat for index access
                
            //     auto& c = corners[i];
            //     std::vector<double> relavtiveCorners {
            //         c[0].x / frameWidth, c[0].y / frameHeight,
            //         c[1].x / frameWidth, c[1].y / frameHeight,
            //         c[2].x / frameWidth, c[2].y / frameHeight,
            //         c[3].x / frameWidth, c[3].y / frameHeight,
            //     };

            //     estimations.emplace_back(m_cameraData, ids[i], theta, pmat, std::move(relavtiveCorners)); 
            // }
            // estimations.shrink_to_fit();
            return std::move(result); 
        }; // Detect

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