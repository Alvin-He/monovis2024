
#include "global.cpp"
#include "const.cpp"
#include "helpers.cpp"
#include <csignal>
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/program_options.hpp>
#include "camera/camera.cpp"
#include "apriltag/apriltag.hpp"
#include "network/publishers.cpp"
#include "network/network_time.cpp"

#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"

#include "conf/parser.cpp"
#include "conf/serviceConfigDef.cpp"
#include "conf/cameraConfigDef.cpp"

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 
namespace PO = boost::program_options;

// maintaines references to states that's needed to run various tasks
struct AppState { // resources should be std::moved into here
    int numCameras = 0;

    std::vector<Conf::CameraConfig> cameraConfigs; 

    // Long live objects, these are just held for ownership and no one really uses them
    std::vector<cv::VideoCapture> cameraCaps; 
};

bool isFlagExit = false; 
cobalt::main co_main(int argc, char* argv[]) {  
    AppState appState {}; 

    // cli argument parsing
    std::string configFilePath; 
    std::string ntServerIP; 
    int cameraID; 
    std::string cameraCalibrationFilePath;

    PO::options_description cliOptions("Command Line Arguments");
    cliOptions.add_options()
        ("help", "show help message")
        ("config,c", 
            PO::value<std::string>(&configFilePath)
            ->default_value("config.toml"), 
            "path to service config file")
        ("nt-ip,S", 
            PO::value<std::string>(&ntServerIP)
            ->default_value("127.0.0.1"), 
            "networktables server ip address")
        ("camera-id,I", 
            PO::value<int>(&cameraID)
            ->default_value(0),
            "camera ID")
        ("camera-calibration-file,F",
            PO::value<std::string>(&cameraCalibrationFilePath)
            ->default_value("calibrationResults_1.xml"), 
            "camera calibration file path")
    ;
    PO::variables_map cliArgMap; 
    PO::store(PO::command_line_parser(argc, argv).options(cliOptions).run(), cliArgMap); 
    PO::notify(cliArgMap); 

    if (cliArgMap.contains("help")) {
        std::cout << cliOptions << std::endl;
        co_return 1; 
    } 

    // toml parsing
    auto config = Conf::LoadToml(configFilePath); 
    Conf::ServiceConfig selfServiceConfig = Conf::ParseServiceConfig(config);
    for (auto file : selfServiceConfig.Cameras_configFiles) {
        appState.cameraConfigs.push_back(Conf::ParseCameraConfig(Conf::LoadToml(file))); 
    }
    
    // global apriltagDetector parameters
    cv::aruco::DetectorParameters APRILTAG_DETECTOR_PARAMS;
    // max smaller than 5 seems to work pretty well
    APRILTAG_DETECTOR_PARAMS.adaptiveThreshWinSizeMin = 5; 
    APRILTAG_DETECTOR_PARAMS.adaptiveThreshWinSizeMax = 5; 
    // APRILTAG_DETECTOR_PARAMS.adaptiveThreshWinSizeStep = 5;
    APRILTAG_DETECTOR_PARAMS.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    // APRILTAG_DETECTOR_PARAMS.useAruco3Detection = true;

    // init network tables
    nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();
    ntInst.StartClient4("monovis"); 
    ntInst.SetServer(selfServiceConfig.Comms_NetworkTable_server.c_str(), selfServiceConfig.Comms_NetworkTable_port);
    NetworkTime::StartPeriodicLatencyUpdate();
    auto appNetworkTable = ntInst.GetTable("SmartDashboard")->GetSubTable("Vision");
    auto robotPosTable = appNetworkTable->GetSubTable("RobotPos"); 
    auto apriltagGroupTable = appNetworkTable->GetSubTable("Apriltags"); 

    auto robotPosePublisher = Publishers::RobotPosePublisher(robotPosTable); 

    Apriltag::World::World robotTracking {K::FIELD}; 
    
    cv::VideoCapture cap{cameraID}; 

    Camera::CameraData cameraData = Camera::LoadCalibDataFromXML(cameraCalibrationFilePath);
    cameraData.id = cameraID; 
    Camera::AdjustCameraDataToForFrameSize(cameraData, cap, K::PROC_FRAME_SIZE);

    Camera::FrameGenerator cameraReader {cap};
    Apriltag::Estimator estimator {cameraData, APRILTAG_DETECTOR_PARAMS};
    
    // signal handlers
    std::signal(SIGINT, [](int i){ isFlagExit = true; }); 
    std::signal(SIGTERM, [](int i){ isFlagExit = true; }); 
    std::signal(SIGABRT, [](int i){ isFlagExit = true; }); 

    // main program loop
    // try {
    fmt::println("starting main program loop");
    while (!isFlagExit)
    {   
        boost::timer::cpu_timer timer; 
        timer.start(); 
        
        cv::Mat frame = co_await cameraReader.PromiseRead(); 
        auto ts = NetworkTime::Now();
        frame = co_await Camera::PromiseResize(frame, K::PROC_FRAME_SIZE);
        #ifdef GUI
        cv::imshow("test", frame);
        #endif
        
        Apriltag::AllEstimationResults res = co_await estimator.PromiseDetect(frame); 
        robotTracking.Update(res);
        fmt::println("time used:{}ms", timer.elapsed().wall/1000000.0); 
        Apriltag::World::RobotPose robotPose = robotTracking.GetRobotPose(); 
        // fmt::println("x: {}, y:{}, r:{}", robotPose.x, robotPose.y, robotPose.rot);
//        fmt::println("distance: {}", std::sqrt(std::pow(robotPose.x, 2) + std::pow(robotPose.y, 2))); 
        co_await robotPosePublisher(Publishers::RobotPosePacket {
            .pose = robotPose, 
            .timestamp = ts
        }); 

        // for (Apriltag::EstimationResult estimation : res) {
        //     fmt::println("rot:{}", estimation.camToTagRvec);
        //     fmt::println("trans:{}", estimation.camToTagTvec); 
        // }
        #ifdef GUI
        cv::waitKey(1);
        #endif
    } 
    // } catch (...) {
    //     std::printf("Exception!"); 
    // };
    
    // exit handling 
    fmt::println("Exiting..."); 
    ntInst.StopClient(); 
    cap.release(); 
    #ifdef GUI
    cv::destroyAllWindows(); 
    #endif
     

    co_return 0; 
}
