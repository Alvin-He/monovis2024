
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


bool isFlagExit = false; 
cobalt::main co_main(int argc, char* argv[]) {  
    // cli argument parsing
    std::string UUID;
    std::string ntServerIP; 
    uint ntServerPort; 
    int cameraID; 
    std::string cameraCalibrationFilePath;

    PO::options_description cliOptions("Command Line Arguments");
    cliOptions.add_options()
        ("help", "show help message")
        ("uuid,u", 
            PO::value<std::string>(&UUID), 
            "service uinque resource ID/name")
        ("nt-ip,S", 
            PO::value<std::string>(&ntServerIP)
            ->default_value("127.0.0.1"), 
            "networktables server ip address")
        ("nt-port,P", 
            PO::value<uint>(&ntServerPort)
            ->default_value(5810), 
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
    ntInst.StartClient4("service-apriltag" + UUID); 
    ntInst.SetServer(ntServerIP.c_str(), ntServerPort);
    NetworkTime::StartPeriodicLatencyUpdate();
    auto ntInternalRoot = ntInst.GetTable("Monovis");

    Publishers::Internal::ApriltagPosePublisher ntApriltagPublisher {UUID, ntInternalRoot}; 

    Apriltag::World::World robotTracking; 
    
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
        #ifdef DEBUG
        boost::timer::cpu_timer timer; 
        timer.start(); 
        #endif 

        cv::Mat frame = co_await cameraReader.PromiseRead(); 
        auto ts = NetworkTime::Now();
        frame = co_await Camera::PromiseResize(frame, K::PROC_FRAME_SIZE);
        
        Apriltag::AllEstimationResults res = co_await estimator.PromiseDetect(frame); 

        std::vector<Publishers::Internal::ApriltagPose> poses;
        for(auto& esti : res) {
            poses.push_back(Publishers::Internal::ApriltagPose {
                .id = esti.id, 
                .pose = Apriltag::World::RobotPoseFromEstimationResult(std::move(esti))
            });
        }
        ntApriltagPublisher(std::move(poses), ts);

        #ifdef DEBUG
        fmt::println("Cycle Time: {}ms", timer.elapsed().wall/1000000);
        #endif
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
