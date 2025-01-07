#pragma once
#include "common.cpp"
#include <vector>
#include "camera/CameraData.cpp"

namespace Apriltag {
    struct EstimationResult {
        std::shared_ptr<Camera::CameraData> cameraInfo; 
        int id; 
        cv::Mat1d camToTagRvec; 
        cv::Mat1d camToTagTvec; 
    };

    class Estimator {
        public: 
        Estimator() {}
        virtual ~Estimator() {}; 

        virtual std::vector<EstimationResult> Detect(cv::Mat image) = 0;
    }; 
}
