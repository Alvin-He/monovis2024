#pragma once
#include "common.cpp"
#include <array>
#include <opencv2/core/types.hpp>
#include <vector>
#include "camera/CameraData.cpp"

namespace Apriltag {
    struct EstimationResult {
        std::shared_ptr<Camera::CameraData> cameraInfo; 
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners; 
    };

    class Estimator {
        public: 
        Estimator() {}
        virtual ~Estimator() {}; 

        virtual EstimationResult Detect(cv::Mat image) = 0;
    }; 
}
