
#include "global.cpp"
#include "helpers.cpp"
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include "camera/camera.cpp"
#include "apriltag/apriltag.hpp"

#include <fmt/include/fmt/ranges.h>
#include <fmt/include/fmt/core.h>
#include <fmt/include/fmt/std.h>

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 

cv::Size2d PROC_FRAME_SIZE(640, 480);

Apriltag::World::WorldInfo FIELD {
    .xTot = 1654, 
    .yTot = 821,
    .tags = Apriltag::World::AllTagsInfo {
        Apriltag::World::TagLocation {.id = 0, .x = 150, .y = 250, .z = 0, .yaw = 180}, 
        Apriltag::World::TagLocation {.id = 1, .x = 1507.9472, .y = 24.5872, .z = 135.5852, .yaw = 120}, 
        Apriltag::World::TagLocation {.id = 2, .x = 0, .y = 0, .z = 0, .yaw = 0}, 
        // Apriltag::World::TagLocation {.id = 2, .x = 1618.5134, .y = 88.2904, .z = 135.5852, .yaw = 120},
        Apriltag::World::TagLocation {.id = 3, .x = 1657.9342, .y = 498.2718, .z = 145.1102, .yaw = 180},
        Apriltag::World::TagLocation {.id = 4, .x = 1657.9342, .y = 218.42, .z = 145.1102, .yaw = 180},
        Apriltag::World::TagLocation {.id = 5, .x = 1470.0758, .y = 820.42, .z = 135.5852, .yaw = 270},
        Apriltag::World::TagLocation {.id = 6, .x = 184.15, .y = 820.42, .z = 135.5852, .yaw = 270},
        Apriltag::World::TagLocation {.id = 7, .x = 0, .y = 554.7868, .z = 145.1102, .yaw = 0}, 
        Apriltag::World::TagLocation {.id = 8, .x = 0, .y = 498.2818, .z = 145.1102, .yaw = 0}, 
        Apriltag::World::TagLocation {.id = 9, .x = 35.6108, .y = 88.3666, .z = 135.5852, .yaw = 60},
        Apriltag::World::TagLocation {.id = 10, .x = 146.1516, .y = 24.5872, .z = 135.5852, .yaw = 60},
        Apriltag::World::TagLocation {.id = 11, .x = 1190.4726, .y = 371.3226, .z = 132.08, .yaw = 300},
        Apriltag::World::TagLocation {.id = 12, .x = 1190.4726, .y = 449.834, .z = 132.08, .yaw = 60},
        Apriltag::World::TagLocation {.id = 13, .x = 1122.0196, .y = 410.5148, .z = 132.08, .yaw = 180},
        Apriltag::World::TagLocation {.id = 14, .x = 532.0792, .y = 410.5148, .z = 132.08, .yaw = 0},
        Apriltag::World::TagLocation {.id = 15, .x = 464.1342, .y = 449.834, .z = 132.08, .yaw = 120},
        Apriltag::World::TagLocation {.id = 16, .x = 464.1342, .y = 371.3226, .z = 132.08, .yaw = 240},
    },
};

cobalt::main co_main(int argc, char* argv[]) {
    printf("Booting up\n"); 
    
    // argument parsing

    Apriltag::World::World robotTracking {FIELD}; 
    // start camera streams
    cv::VideoCapture cap{0}; 
    cobalt::generator<cv::Mat> cameraReader = Camera::Reader(cap); 
    Apriltag::Estimator estimator{}; 
    while (true)
    {
        cv::Mat frame = co_await cameraReader; 
        frame = co_await Camera::CudaResize(frame, PROC_FRAME_SIZE); 
        #ifdef GUI
        cv::imshow("test", frame);
        #endif
        boost::timer::cpu_timer timer; 
        timer.start(); 
        Apriltag::AllEstimationResults res = co_await estimator.Detect(frame); 
        robotTracking.Update(res);
        // fmt::println("time used:{}ms", timer.elapsed().wall/1000000); 
        // for (Apriltag::EstimationResult estimation : res) {
        //     fmt::println("rot:{}", estimation.camToTagRvec);
        //     fmt::println("trans:{}", estimation.camToTagTvec); 
        // }
        #ifdef GUI
        cv::waitKey(1);
        #endif
    }
    

    // start processing tasks
    
    co_return 0; 
}