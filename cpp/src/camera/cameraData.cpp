#pragma once 

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
    };

    // deseralize camera calibration data from xml file
    CameraData LoadCalibDataFromXML(const std::string& filePath) {
        cv::FileStorage cameraCalibData {filePath, cv::FileStorage::READ}; 
        cv::Mat matrix; 
        cameraCalibData["cameraMatrix"] >> matrix;  
        cv::Mat distCoeffs;
        cameraCalibData["dist_coeffs"] >> distCoeffs; 
        cameraCalibData.release(); 
        CameraData cameraMainData = {
            .matrix = std::move(matrix), 
            .distCoeffs = std::move(distCoeffs)
        }; 
        return std::move(cameraMainData); 
    }

    // adjust camera matrix for resized smaller image
    void AdjustCameraDataToForFrameSize(CameraData& cameraData, const cv::Size2d& originalFrameSize, const cv::Size2d& targetFrameSize) {
        cameraData.matrix(0, 0) *= (targetFrameSize.width / originalFrameSize.width); // fx
        cameraData.matrix(0, 2) *= (targetFrameSize.width / originalFrameSize.width); // cx
        cameraData.matrix(1, 1) *= (targetFrameSize.height / originalFrameSize.height); // fy
        cameraData.matrix(1, 2) *= (targetFrameSize.height / originalFrameSize.height); // cy
    }

    // AdjustCameraDataToForFrameSize but directly getting the originalFrameSize directly from a VideoCapture
    void AdjustCameraDataToForFrameSize(CameraData& cameraData, const cv::VideoCapture& cap, const cv::Size2d& targetFrameSize) {
        cv::Size2d originalFrameSize(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT)); 
        AdjustCameraDataToForFrameSize(cameraData, originalFrameSize, targetFrameSize); 
    }
} // namespace Camera
