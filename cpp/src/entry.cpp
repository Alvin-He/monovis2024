
#include "global.cpp"
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include "camera/camera.cpp"
#include "apriltag/apriltag.hpp"

#include <fmt/include/fmt/ranges.h>

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 

cv::Size2d PROC_FRAME_SIZE(640, 480);

cobalt::main co_main(int argc, char* argv[]) {
    printf("Booting up\n"); 
    
    // argument parsing

    // start camera streams
    cv::VideoCapture cap{0}; 
    cobalt::generator<cv::Mat> cameraReader = Camera::Reader(cap); 
    Apriltag::Estimator estimator{}; 
    while (true)
    {
        cv::Mat frame = co_await cameraReader; 
        frame = co_await Camera::CudaResize(frame, PROC_FRAME_SIZE); 
        cv::imshow("test", frame);
        Apriltag::AllEstimations res = co_await estimator.Detect(frame); 
        for (Apriltag::Estimation estimation : res) {
            fmt::println("rot:\n{}", estimation.rot); 
        }
        cv::waitKey(1);
    }
    

    // start processing tasks
    
    co_return 0; 
}