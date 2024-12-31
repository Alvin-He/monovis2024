#include "fmt/core.h"
#include "global.cpp"
#include "const.cpp"
#include "helpers.cpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/program_options.hpp>
#include <exception>
#include <memory>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_cpp.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/videoio.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include "camera/Camera.cpp"
#include "camera/CameraData.cpp"
#include "apriltag/OpenCVArucoEstimator.cpp"
#include "network/ApriltagPose.cpp"
#include "network/network_time.cpp"
#include "network/redis.cpp"
#include "program_options/value_semantic.hpp"
#include "world/World.cpp"
#include "fmt/include/fmt/chrono.h"
#include "network/ReadNetworkFrame.cpp"

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 
namespace PO = boost::program_options;

cobalt::main co_main(int argc, char* argv[]) {  
    // cli argument parsing
    std::string UUID;
    std::string cameraUUID;
    std::string configFolderPath;
    std::string serverIP = "default"; 
    uint serverPort = 0; 
    bool useNT = false;

    PO::options_description cliOptions("Command Line Arguments");
    cliOptions.add_options()
        ("help", "show help message")
        ("config,c", 
            PO::value<std::string>(&configFolderPath)
            ->default_value("./"), 
            "path to config folder, default current working directory")
        ("camera-uuid,u", 
            PO::value<std::string>(&UUID)
            ->required(),
            "cameras UUID to use, this uuid must match the uuids provided by camera-config-file")
        ("ip,s", 
            PO::value<std::string>(&serverIP),
            "apriltag publish server ip address (redis/NT) depending on useNT. default to default addresses")
        ("port,p", 
            PO::value<uint>(&serverPort),
            "apriltag publish server port (redis/NT) depending on useNT. default to default ports")
        ("useNT,T", 
            PO::value<bool>(&useNT)
            ->default_value(true),
            "use NetworkTables for transport instead of redis, only use in single camera set ups! default: true"
        )
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
    APRILTAG_DETECTOR_PARAMS.cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
    APRILTAG_DETECTOR_PARAMS.relativeCornerRefinmentWinSize = 0.5;
    APRILTAG_DETECTOR_PARAMS.cornerRefinementWinSize = 10;
    // APRILTAG_DETECTOR_PARAMS.relativeCornerRefinmentWinSize = 0.7;
    APRILTAG_DETECTOR_PARAMS.minCornerDistanceRate *= 0.05;
    APRILTAG_DETECTOR_PARAMS.minMarkerDistanceRate *= 0.05; 
    APRILTAG_DETECTOR_PARAMS.minMarkerPerimeterRate *= 0.05;
    // APRILTAG_DETECTOR_PARAMS.useAruco3Detection = true;
    // APRILTAG_DETECTOR_PARAMS.minSideLengthCanonicalImg = 13;    

    // APRILTAG_DETECTOR_PARAMS.useAruco3Detection = true;

    // init RedisDB access if enabled
    if (!useNT) {
        throw std::runtime_error("redis is currently unusuable");
        // serverIP = serverIP != "default" ? serverIP : "127.0.0.1"; 
        // serverPort = serverPort != -1 ? serverPort : 6379; 
        // co_await RedisDB::init(serverIP, std::to_string(serverPort), camerasTomlPath); 
        // co_await RedisDB::ping();
    }else{
    // init network tables if enabled
        serverIP = serverIP != "default" ? serverIP : "127.0.0.1"; 
        // don't need to fix server port because port of 0 for the NTcore api default to the default port
        nt::NetworkTableInstance ntRio = nt::NetworkTableInstance::GetDefault();
        ntRio.StartClient4("monovis-pose-" + UUID);
        ntRio.SetServer(serverIP.c_str(), serverPort); 
    }


    Camera::PopulateGlobalCameraRegistraFromTOML(
        Conf::LoadToml(fmt::format("{}/cameras.toml", configFolderPath))
    );
    auto cdi = Camera::GlobalCameraRegistra.find(UUID);
    if (cdi == Camera::GlobalCameraRegistra.end()) throw std::runtime_error("provided uuid isn't found in cameras.toml");
    auto cameraData = cdi->second;    

    // bootstrap camera
    cv::VideoCapture cap{cameraData->id}; 

    Camera::AdjustCameraDataAndCapture(cameraData, cap);     
    Camera::AdjustCameraDataForNewImageSize(cameraData, cameraData->calibratedAspectRatio, K::PROC_FRAME_SIZE);

    Camera::FrameGenerator cameraReader {cap};
    // for (auto& buf : frameBufs) {
    //     cap.read(buf);
    // }
    // cameraRead(cap, readReq, readFinished).detach(); 

    fmt::println("camera {} initiated", cameraData->id);

    // construct estimator
    Apriltag::OpenCVArucoEstimator estimator {cameraData, APRILTAG_DETECTOR_PARAMS};

    // construct publisher
    std::unique_ptr<Network::ApriltagPose::Publisher> apriltagPublisher; 
    if (useNT) apriltagPublisher = std::make_unique<Network::ApriltagPose::NTPublisher>(UUID);
    // else apriltagPublisher = std::make_unique<Network::ApriltagPose::RedisPublisher>(UUID);

    // World::RobotPose robotTracking; 

    // signal handlers
    std::signal(SIGINT, [](int i){ f_exit = true; }); 
    std::signal(SIGTERM, [](int i){ f_exit = true; }); 
    std::signal(SIGABRT, [](int i){ f_exit = true; }); 

    

    // main program loop
    // try {
    fmt::println("starting main program loop");
    while (!f_exit)
    {   
        // fmt::println("LS");
        #ifdef DEBUG
        auto start = std::chrono::high_resolution_clock::now();
        #endif 
        // cv::Mat frame; 
        // if (auto ret = co_await Network::ReadFrame("127.0.0.1", "8081")) {
        //     frame = ret.value(); 
        // } else {
        //     continue;
        // }
        cv::Mat frame = co_await cameraReader.PromiseRead();
        // cv::Mat frame = cv::imread("/mnt/1ECC5E47CC5E18FB/Users/alh/Desktop/monovis2024/frontend/My project/testImg1.png");

        // cv::Mat frame = co_await getFrame();
        // fmt::print("\tREAD: {}", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start));

        auto ts = NetworkTime::Now();
        frame = Camera::Resize(frame, K::PROC_FRAME_SIZE);
        // frame = co_await Camera::PromiseResize(frame, K::PROC_FRAME_SIZE);
        // fmt::print("\tRESIZE: {}", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start));

        // cv::Mat undistored;
        // cv::undistort(frame, undistored, s_cameraData->matrix, s_cameraData->distCoeffs);
        // cv::imshow("undistored", undistored); 

        std::vector<Apriltag::EstimationResult> res =  estimator.Detect(frame); //co_await estimator.PromiseDetect(frame); 
        // Apriltag::AllEstimationResults res = estimator.Detect(frame); 
        // fmt::print("\tDETECT: {}", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start));
        
        #ifdef DEBUG 
        for(auto& esti : res) {
            auto lastPose = World::Solvers::RobotPoseFromEstimationResult(esti); 
            fmt::print("Tag {}, x {:.2f}, y {:.2f}, r {:.2f}\t", lastPose.id, 
                lastPose.x, lastPose.y, lastPose.rot);
        }
        #endif

        // fmt::print("\tESTIMATE: {}", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start));

        co_await apriltagPublisher->publish(res, ts);
        // fmt::print("\tPUBLISH: {}", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start));
        // robotTracking.Update(res);
        // auto pose = robotTracking.GetRobotPose();
        // fmt::println("{}, {} r:{}", pose.x, pose.y, pose.rot);
        #ifdef DEBUG
        fmt::print("\tTotal Time: {}", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start));
        fmt::println("");
        
        #endif
        #ifdef GUI
        cv::waitKey(1);
        #endif
        // cv::waitKey();
        // break;
    } 
    // } catch (...) {
    //     std::printf("Exception!"); 
    // };
    
    // exit handling 
    fmt::println("Exiting..."); 

    cap.release(); 
    #ifdef GUI
    cv::destroyAllWindows(); 
    #endif

    co_return 0; 
}
