
#include "global.cpp"
#include "helpers.cpp"
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/program_options.hpp>
#include "camera/camera.cpp"
#include "apriltag/apriltag.hpp"
#include "network/publishers.cpp"

#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 
namespace PO = boost::program_options;

cv::Size2d PROC_FRAME_SIZE (640, 480); 

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
    // argument parsing
    std::string ntServerIP; 
    int cameraID; 
    std::string cameraCalibrationFilePath;

    PO::options_description cliOptions("Command Line Arguments");
    cliOptions.add_options()
        ("help", "show help message")
        ("nt-ip,S", 
            PO::value<std::string>(&ntServerIP)
            ->default_value("127.0.0.1"), 
            "networktables server ip address")
        ("camera-id,I", 
            PO::value<int>(&cameraID)
            ->default_value(0),
            "camera ID")
        ("camera-calibration-file,CCF",
            PO::value<std::string>(&cameraCalibrationFilePath)
            ->default_value("calibrationResults_2.xml"), 
            "camera calibration file path")
    ;
    PO::variables_map cliArgMap; 
    PO::store(PO::command_line_parser(argc, argv).options(cliOptions).run(), cliArgMap); 
    PO::notify(cliArgMap); 

    if (cliArgMap.contains("help")) {
        std::cout << cliOptions << std::endl;
        co_return 1; 
    } 

    nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();
    ntInst.StartClient4("monovis"); 
    ntInst.SetServer(ntServerIP.c_str(), 5810);
    auto appNetworkTable = ntInst.GetTable("SmartDashboard")->GetSubTable("Vision");
    auto robotPosTable = appNetworkTable->GetSubTable("RobotPos"); 
    auto apriltagGroupTable = appNetworkTable->GetSubTable("Apriltags"); 

    auto robotPosePublisher = Publishers::RobotPosePublisher(robotPosTable); 

    Apriltag::World::World robotTracking {FIELD}; 
    // deseralize camera calibration data
    // cv::FileStorage cameraCalibData {"calibrationResults_1_0.498.xml", cv::FileStorage::READ}; 
    cv::FileStorage cameraCalibData {cameraCalibrationFilePath, cv::FileStorage::READ}; 
    cv::Mat matrix; 
    cameraCalibData["cameraMatrix"] >> matrix;  
    cv::Mat distCoeffs;
    cameraCalibData["dist_coeffs"] >> distCoeffs; 
    cameraCalibData.release(); 
    Apriltag::CameraData cameraMainData = {
        .matrix = std::move(matrix), 
        .distCoeffs = std::move(distCoeffs)
    }; 

    // start camera streams
    cv::VideoCapture cap{cameraID}; 

    // adjust camera matrix for resized smaller image
    cameraMainData.matrix(0, 0) *= (PROC_FRAME_SIZE.width / cap.get(cv::CAP_PROP_FRAME_WIDTH)); // fx
    cameraMainData.matrix(0, 2) *= (PROC_FRAME_SIZE.width / cap.get(cv::CAP_PROP_FRAME_WIDTH)); // cx
    cameraMainData.matrix(1, 1) *= (PROC_FRAME_SIZE.height / cap.get(cv::CAP_PROP_FRAME_HEIGHT)); // fy
    cameraMainData.matrix(1, 2) *= (PROC_FRAME_SIZE.height / cap.get(cv::CAP_PROP_FRAME_HEIGHT)); // cy

    cobalt::generator<cv::Mat> cameraReader = Camera::Reader(cap); 
    Apriltag::Estimator estimator {cameraMainData}; 
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
        fmt::println("time used:{}ms", timer.elapsed().wall/1000000.0); 
        Apriltag::World::RobotPose robotPose = robotTracking.GetRobotPose(); 
        fmt::println("x: {}, y:{}, r:{}", robotPose.x, robotPose.y, robotPose.rot);
        fmt::println("distance: {}", std::sqrt(std::pow(robotPose.x, 2) + std::pow(robotPose.y, 2))); 
        co_await robotPosePublisher(robotPose); 

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
