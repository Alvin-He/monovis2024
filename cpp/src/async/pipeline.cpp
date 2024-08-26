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
    std::shared_ptr<int64_t> frameTimeStamp; 
    std::shared_ptr<Apriltag::AllEstimationResults> estimationResult; 

};


void NormalCameraPipeline(std::shared_ptr<State> statePtr) { cobalt::run( ([&]() -> cobalt::task<void> {
    State state = *statePtr; 
    boost::asio::steady_timer execTimer {co_await cobalt::this_coro::executor, K::APRILTAG_LOOP_UPDATE_INTERVAL}; 
    for(;;) { boost::this_thread::interruption_point(); // allow interrupts 
        boost::timer::cpu_timer timer;

        auto frame = co_await state.frameGen.PromiseRead();
        frame = co_await Camera::PromiseResize(frame, K::PROC_FRAME_SIZE); 
        state.frameTimeStamp = std::make_shared<int64_t>(NetworkTime::Now()); 

        if (state.config.Camera_apriltagEnabled) {
            auto estimatorRes = co_await state.estimator.PromiseDetect(frame); 
            state.estimationResult = std::make_shared<Apriltag::AllEstimationResults>(std::move(estimatorRes)); 
        }
        fmt::println("time used:{}ms", timer.elapsed().wall/1000000.0); 

        co_await execTimer.async_wait(cobalt::use_op);
        execTimer.expires_at(execTimer.expires_at() + K::APRILTAG_LOOP_UPDATE_INTERVAL);
    };
})());}; 