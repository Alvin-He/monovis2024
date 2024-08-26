#pragma once

#include "common.cpp"
#include "global.cpp"

#include <boost/thread.hpp>
#include "opencv4/opencv2/opencv.hpp"

#include "apriltag/apriltag.hpp"
#include "conf/cameraConfigDef.cpp"
#include "camera/camera.cpp"
#include "camera/cameraData.cpp"

#include "network/network_time.cpp"

#include <boost/timer/timer.hpp>

struct State {
    Conf::CameraConfig config;
    Camera::CameraData cameraData;

    Apriltag::Estimator estimator; 
    Camera::FrameGenerator frameGen; 

    // these shared pointers must be replaced with assignment operator!!! to avoid locking issues
    // they will be freed automatically when no one is using them
    std::shared_ptr<std::shared_ptr<int64_t>> frameTimeStamp; 
    std::shared_ptr<std::shared_ptr<Apriltag::AllEstimationResults>> estimationResult; 

};


void NormalCameraPipeline(std::shared_ptr<State> statePtr) {
    State state = *statePtr;
    auto lastTime = boost::chrono::system_clock::now();
    for(;;) { boost::this_thread::interruption_point(); // allow interrupts 
        boost::timer::cpu_timer timer;
    
        auto frame = state.frameGen.Read();
        frame = Camera::Resize(frame, K::PROC_FRAME_SIZE); 
        state.frameTimeStamp = std::make_shared<std::shared_ptr<int64_t>>(std::make_shared<int64_t>(NetworkTime::Now())); 

        if (state.config.Camera_apriltagEnabled) {
            // auto estimatorRes = co_await state.estimator.PromiseDetect(frame); 
            auto estimatorRes = state.estimator.Detect(frame); 
            state.estimationResult = std::make_shared<std::shared_ptr<Apriltag::AllEstimationResults>>(std::make_shared<Apriltag::AllEstimationResults>(std::move(estimatorRes))); 
        }
        fmt::println("time used:{}ms", timer.elapsed().wall/1000000.0); 

        auto endTime = boost::chrono::system_clock::now();
        auto waitDuration = endTime - (lastTime + K::APRILTAG_LOOP_UPDATE_INTERVAL); 
        lastTime = endTime; 
        if (waitDuration > boost::chrono::milliseconds(0)) boost::this_thread::sleep_for(waitDuration); 
    };
};