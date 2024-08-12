#pragma once
#include "global.cpp"
#include "common.cpp"
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "helpers.cpp"
#include "camera/cameraData.cpp"


namespace Apriltag {

struct EstimationResult {
    Camera::CameraData cameraInfo; 
    int id; 
    cv::Mat1d camToTagRvec; 
    cv::Mat1d camToTagTvec; 
};
typedef std::vector<EstimationResult> AllEstimationResults; 

class Estimator {
    public:
        Estimator(Camera::CameraData cameraData, cv::aruco::DetectorParameters detectorParams): 
            m_cameraData(std::move(cameraData)), 
            m_detector(
                cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), detectorParams) 
        {
            
        }; // Estimator

        AllEstimationResults Detect(cv::Mat image) {
            AllEstimationResults estimations; 

            // detecting the tags in image
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids; 
            this->m_detector.detectMarkers(image, corners, ids); 
            
            #if defined(DEBUG) && defined(GUI)
            cv::aruco::drawDetectedMarkers(image, corners, ids); 
            cv::imshow(fmt::format("camera-{} markers", m_cameraData.id), image); 
            #endif
            // solvepnp to generate cords
            int maxI = ids.size();
            for (int i = 0; i < maxI; i++) {
                cv::Mat1d rvec;
                cv::Mat1d tvec; 
                cv::solvePnP(
                    m_objectPoints, 
                    corners[i], 
                    m_cameraData.matrix, 
                    m_cameraData.distCoeffs,
                    rvec, 
                    tvec); 
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

                estimations.push_back(EstimationResult {
                    .cameraInfo = m_cameraData,
                    .id = ids[i],
                    .camToTagRvec = theta, 
                    .camToTagTvec = pmat
                }); 
            }
            return std::move(estimations); 
        }; // Detect

        cobalt::promise<AllEstimationResults> PromiseDetect(cv::Mat image) {
            co_return std::move(Detect(image)); 
        }
        cobalt::task<AllEstimationResults> TaskDetect(cv::Mat image) {
            co_return std::move(Detect(image)); 
        }

    private:
        Camera::CameraData m_cameraData; 
        cv::aruco::ArucoDetector m_detector;
        std::vector<cv::Point3f> m_objectPoints = {
            {-4, 4, 0}, // every one is 33 mm
            { 4, 4, 0},
            { 4,-4, 0},
            {-4,-4, 0}
        };

};
}// namespace Apriltag