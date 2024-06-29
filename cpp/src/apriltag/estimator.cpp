#pragma once
#include "global.cpp"
#include "common.cpp"
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "helpers.cpp"

namespace Apriltag {
struct CameraData {
  int id = 0; 
  bool apriltags = true;
  // int camToRobotPos{4} = {800,250,300, 0}, # anchored at bottom right of robot
  double camToRobotPos[4] = {0,0,0, 0};
  // std::vector<std::vector<double>> matrix =
  //   {{710.8459662, 0, 584.09769116},
  //   {0.,710.64515618, 485.94212983},
  //   {0., 0., 1., }},
  // std::vector<std::vector<double>> distCoeffs =
  // {{-0.3689181,0.12470983,-0.0062236,0.00298559,-0.01839474}};
  cv::Matx<double, 3, 3> matrix = {673.49634849, 0, 616.93113106, 0, 670.71012973, 536.45109056, 0, 0, 1};
    // {{673.49634849, 0, 616.93113106},
    // {0, 670.71012973, 536.45109056},
    // {0, 0, 1}};
  cv::Matx<double, 1, 5> distCoeffs = 
    {-0.18422303, 0.04338743, -0.0010019, 0.00080675, -0.00543398};
    // {{-0.18422303, 0.04338743, -0.0010019, 0.00080675, -0.00543398}};
};

struct EstimationResult {
    const CameraData& cameraInfo; 
    int id; 
    cv::Mat1d camToTagRvec; 
    cv::Mat1d camToTagTvec; 
};
typedef std::vector<EstimationResult> AllEstimationResults; 

class Estimator {
    public:
        Estimator(CameraData cameraData): 
            m_cameraData(std::move(cameraData)), 
            // m_detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)) 
            m_dictionary(std::make_shared<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11))) 
        {
            
        }; // Estimator

        cobalt::promise<AllEstimationResults> Detect(cv::Mat image) {
            AllEstimationResults estimations; 

            // detecting the tags in image
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids; 
            cv::aruco::detectMarkers(image, this->m_dictionary, corners, ids);
            // this->m_detector.detectMarkers(image, corners, ids); 
            
            #if defined(DEBUG) && defined(GUI)
            cv::aruco::drawDetectedMarkers(image, corners, ids); 
            cv::imshow("markers", image); 
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
            co_return std::move(estimations); 
        }; // Detect


    private:
        CameraData m_cameraData; 
        // cv::aruco::ArucoDetector m_detector;
        std::shared_ptr<cv::aruco::Dictionary> m_dictionary; 
        // cv::aruco::Dictionary m_dictionary;  
        std::vector<cv::Point3f> m_objectPoints = {
            {-4, 4, 0}, // every one is 33 mm
            { 4, 4, 0},
            { 4,-4, 0},
            {-4,-4, 0}
        };

};
}// namespace Apriltag