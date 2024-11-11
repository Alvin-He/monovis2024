#pragma once 

#include "common.cpp"
#include <opencv2/core/types.hpp>
#include <opencv4/opencv2/opencv.hpp>

namespace Camera
{
    struct CameraData {
        int id = 0; 
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
        cv::Size2d calibratedAspectRatio = cv::Size2d(1280, 720);
    };

    // deseralize camera calibration data from xml file
    CameraData LoadCalibDataFromXML(const std::string& filePath) {
        cv::FileStorage cameraCalibData {filePath, cv::FileStorage::READ}; 
        cv::Mat matrix; 
        cameraCalibData["cameraMatrix"] >> matrix;  
        cv::Mat distCoeffs;
        cameraCalibData["dist_coeffs"] >> distCoeffs;
        cv::Size2d size; 
        cameraCalibData["cameraResolution"] >> size; 
        cameraCalibData.release(); 
        CameraData cameraMainData = {
            .matrix = std::move(matrix), 
            .distCoeffs = std::move(distCoeffs),
            .calibratedAspectRatio = std::move(size)
        }; 
        return std::move(cameraMainData); 
    }
    // Modifiys the capture aspect ratio to the calibrated aspect ratio and rescales the camera matrix for a desired frame size
    void AdjustCameraDataAndCapture(CameraData& cameraData, cv::VideoCapture& cap, const cv::Size2d& targetFrameSize) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, cameraData.calibratedAspectRatio.width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, cameraData.calibratedAspectRatio.height);
       
        cameraData.matrix = cv::getOptimalNewCameraMatrix(cameraData.matrix, cameraData.distCoeffs, cameraData.calibratedAspectRatio, 1, targetFrameSize);    
    }
} // namespace Camera
