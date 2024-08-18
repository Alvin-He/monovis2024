#pragma once 

#include "common.cpp"
#include "global.cpp"

#include "async/synchronization.cpp"
#include "async/task.cpp"
#include "async/macro.cpp"

#include "opencv4/opencv2/opencv.hpp"

#include "apriltag/apriltag.hpp"
#include "conf/cameraConfigDef.cpp"
#include "camera/camera.cpp"
#include "camera/cameraData.cpp"

#include "network/network_time.cpp"

#include <boost/timer/timer.hpp>

namespace cobalt = boost::cobalt; 
namespace asio = boost::asio; 

struct Async::State {
    Async::SharedRes<Conf::CameraConfig> config;
    Async::SharedRes<Camera::CameraData> cameraData;

    Async::AtomicRes<Apriltag::Estimator> estimator;
    Async::SharedRes<Camera::FrameGenerator> frameGen;

    Async::SharedRes<cv::Mat> frame; 
    Async::SharedRes<int64_t> frameTimeStamp; 
    Async::SharedRes<Apriltag::AllEstimationResults> estimationResults;
};

namespace Tasks {
TASK(PreprocessCameraFrames) {
    // lock all the resources used
    std::shared_lock _l1 {state->cameraData}; 
    std::shared_lock _l4 {state->frameGen}; 
    std::unique_lock _l2 {state->frame}; 
    std::unique_lock _l3 {state->frameTimeStamp}; 

    // get frame
    auto newFrame = co_await state->frameGen->TaskRead(); 
    *state->frameTimeStamp = NetworkTime::Now(); // get timestamp
    *state->frame = co_await Camera::TaskResize(newFrame, K::PROC_FRAME_SIZE); // resize image (with cuda)
}; 

TASK(EstimateApriltag) {
    std::unique_lock _l1 {state->estimationResults}; 
    std::shared_lock _l2 {state->frame}; 

    boost::timer::cpu_timer timer;
    timer.start(); 
    // run detection
    *state->estimationResults = co_await state->estimator->TaskDetect(*state->frame); 
    fmt::println("time used:{}ms", timer.elapsed().wall/1000000.0); 
}; 
}


