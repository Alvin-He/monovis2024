#pragma once 

#include "common.cpp"
#include <array>
#include <map>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include "conf/parser.cpp"

namespace Camera
{
    struct CameraData {
        // Any changes to this struct that affects parsing need to update conf/Parse_cameras_toml
        std::string uuid; 
        int deviceID = 0; 
        // int camToRobotPos{4} = {800,250,300, 0}, # anchored at bottom right of robot
        std::array<double, 4> cameraPos = {0,0,0, 270};
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
        cv::Size2d resizedTo = cv::Size2d(1280, 720);
    };

    // uuid, asosociated camera data
    static std::map<std::string, std::shared_ptr<CameraData>> GlobalCameraRegistra;

    // adjust camera matrix for resized smaller image
    void AdjustCameraDataForNewImageSize(std::shared_ptr<CameraData> cameraData, const cv::Size2d& originalFrameSize, cv::Size2d targetFrameSize) {
        cameraData->matrix(0, 0) *= (targetFrameSize.width / originalFrameSize.width); // fx
        cameraData->matrix(0, 2) *= (targetFrameSize.width / originalFrameSize.width); // cx
        cameraData->matrix(1, 1) *= (targetFrameSize.height / originalFrameSize.height); // fy
        cameraData->matrix(1, 2) *= (targetFrameSize.height / originalFrameSize.height); // cy

        cameraData->resizedTo = targetFrameSize;
    }

    // Modifiys the capture aspect ratio to the calibrated aspect ratio and rescales the camera matrix for a desired frame size
    void AdjustCameraCaptureToNewImageSize(std::shared_ptr<CameraData> cameraData, cv::VideoCapture& cap) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, cameraData->calibratedAspectRatio.width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, cameraData->calibratedAspectRatio.height);
    }


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

    CameraData ParseCameraData(const toml::table& cameraDataTable) {
        auto calibrationFile = Conf::parse<std::string>(cameraDataTable, "calibration_file"); 
        
        auto res = LoadCalibDataFromXML(calibrationFile);

        res.deviceID = Conf::parse<int>(cameraDataTable, "device_id");
        res.cameraPos = {
            Conf::parse<double>(cameraDataTable, "x"),
            Conf::parse<double>(cameraDataTable, "y"),
            Conf::parse<double>(cameraDataTable, "z"),
            Conf::parse<double>(cameraDataTable, "r")
        };

        return res;
    }

    void PopulateGlobalCameraRegistraFromTOML(const toml::table& cameras_toml) {
        for (auto& [k, v] : cameras_toml) {
            auto uuid = k.str();
            if (!v.is_table()) throw std::runtime_error(fmt::format("uuid: {} is not a table", uuid)); 
            auto tab = *v.as_table();
            
            auto d = std::make_shared<CameraData>(ParseCameraData(tab));
            d->uuid = std::move(std::string{uuid});

            // reset the camera to the processing frame size
            AdjustCameraDataForNewImageSize(d, d->calibratedAspectRatio, K::PROC_FRAME_SIZE); 

            GlobalCameraRegistra.insert_or_assign(std::string{uuid}, d); 
        }
    }
} // namespace Camera
