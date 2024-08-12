
#pragma once 


#include "parser.cpp"

namespace Conf {
struct CameraConfig {
    //Camera
    int Camera_id; // opencv needs int 
    std::string Camera_calibrationFile; 
    
    bool Camera_apriltagEnabled; 
    bool Camera_poseEstimationEnabled; 
}; 

CameraConfig ParseCameraConfig(const toml::table& configTable) {
    return CameraConfig {
        .Camera_id = Conf::parse<int>(configTable, "Camera.id"), 
        .Camera_calibrationFile = Conf::parse<std::string>(configTable, "Camera.calibrationFile"), 
        .Camera_apriltagEnabled = Conf::parse<bool>(configTable, "Camera.apriltag"), 
        .Camera_poseEstimationEnabled = Conf::parse<bool>(configTable, "Camera.poseEstimation")
    };
}
}