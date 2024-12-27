
#include "common.cpp"
#include "const.cpp"
#include <array>
#include <chrono>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <unistd.h>
#include "fmt/include/fmt/chrono.h"
#include <boost/asio/steady_timer.hpp>

#include "camera/CameraData.cpp"

cobalt::main co_main(int argc, char* argv[]) {

    std::array<cv::Point3f, 4> objectPoints = {
        cv::Point3f {-4, 4, 0},
        cv::Point3f { 4, 4, 0},
        cv::Point3f { 4,-4, 0},
        cv::Point3f {-4,-4, 0}
    };

    std::array<cv::Point2f, 4> imgPoints = {
        cv::Point2f {50, 100}, 
        cv::Point2f {25,68}, 
        cv::Point2f {50, 68}, 
        cv::Point2f {25, 100}
    }; 


    Camera::CameraData caemraData;
    
    cv::Mat rvec; 
    cv::Mat tvec;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        cv::solvePnP(objectPoints, imgPoints, caemraData.matrix,  caemraData.distCoeffs, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
        
        auto delta = std::chrono::high_resolution_clock::now() - start; 
        fmt::println("took {}", std::chrono::duration_cast<std::chrono::milliseconds>(delta)); 

        boost::asio::steady_timer tim{co_await asio::this_coro::executor, std::chrono::milliseconds(1)}; 
        co_await tim.async_wait(cobalt::use_op);
    }
    
    // std::terminate();
    co_return 0;
}