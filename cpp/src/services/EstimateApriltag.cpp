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
#include <string>
#include "camera/Camera.cpp"
#include "camera/CameraData.cpp"
#include "apriltag/OpenCVArucoEstimator.cpp"
#include "network/ApriltagPose.cpp"
#include "network/network_time.cpp"
#include "network/redis.cpp"
#include "program_options/value_semantic.hpp"
#include "worldPose/World.cpp"
#include "fmt/include/fmt/chrono.h"

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 
namespace PO = boost::program_options;

cobalt::main co_main(int argc, char* argv[]) {  
    // cli argument parsing
    std::string UUID;
    std::string serverIP = "default"; 
    uint serverPort = 0; 
    bool useNT = false;
    int cameraID = 0; 
    std::string cameraCalibrationFilePath;

    PO::options_description cliOptions("Command Line Arguments");
    cliOptions.add_options()
        ("help", "show help message")
        ("uuid,u", 
            PO::value<std::string>(&UUID)
            ->default_value("monovis-unnamed"), 
            "service uinque resource ID/name")
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
        serverIP = serverIP != "default" ? serverIP : "127.0.0.1"; 
        serverPort = serverPort != -1 ? serverPort : 6379; 
        co_await RedisDB::init(serverIP, std::to_string(serverPort), UUID); 
        co_await RedisDB::ping();
    }else{
    // init network tables if enabled
        serverIP = serverIP != "default" ? serverIP : "127.0.0.1"; 
        // don't need to fix server port because port of 0 for the NTcore api default to the default port
        nt::NetworkTableInstance ntRio = nt::NetworkTableInstance::GetDefault();
        ntRio.StartClient4("monovis-pose-" + UUID);
        ntRio.SetServer(serverIP.c_str(), serverPort); 
    }

    // bootstrap camera
    cv::VideoCapture cap{cameraID}; 

    // static to ensure cameraData always exists 
    static Camera::CameraData cameraData = Camera::LoadCalibDataFromXML(cameraCalibrationFilePath);
    cameraData.id = cameraID; 

    Camera::AdjustCameraDataAndCapture(cameraData, cap);     
    Camera::AdjustCameraDataForNewImageSize(cameraData, cameraData.calibratedAspectRatio, K::PROC_FRAME_SIZE);

    Camera::FrameGenerator cameraReader {cap};
    // for (auto& buf : frameBufs) {
    //     cap.read(buf);
    // }
    // cameraRead(cap, readReq, readFinished).detach(); 
    std::shared_ptr<Camera::CameraData> s_cameraData (&cameraData); 

    fmt::println("camera {} initiated", cameraID);

    // construct estimator
    Apriltag::OpenCVArucoEstimator estimator {s_cameraData, APRILTAG_DETECTOR_PARAMS};

    // construct publisher
    std::unique_ptr<Network::ApriltagPose::Publisher> apriltagPublisher; 
    if (useNT) apriltagPublisher = std::make_unique<Network::ApriltagPose::NTPublisher>("");
    // else apriltagPublisher = std::make_unique<Network::ApriltagPose::RedisPublisher>(UUID);

    // signal handlers
    std::signal(SIGINT, [](int i){ f_exit = true; }); 
    std::signal(SIGTERM, [](int i){ f_exit = true; }); 
    std::signal(SIGABRT, [](int i){ f_exit = true; }); 
    WorldPose::World robotTracking; 
    // main program loop
    // try {
    fmt::println("starting main program loop");
    while (!f_exit)
    {   
        // fmt::println("LS");
        #ifdef DEBUG
        auto start = std::chrono::high_resolution_clock::now();
        #endif 

        // cv::Mat frame = co_await cameraReader.PromiseRead();
        cv::Mat frame = cv::imread("/mnt/1ECC5E47CC5E18FB/Users/alh/Desktop/monovis2024/frontend/My project/testImg1.png");

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
            auto lastPose = WorldPose::Solvers::RobotPoseFromEstimationResult(esti); 
            fmt::print("Tag {}, dist {:.2f}, x {:.2f}, y {:.2f}, r {:.2f}\t", lastPose.id, 
                std::sqrt(std::pow(lastPose.x, 2) + std::pow(lastPose.y,2)), // pathegram formula distance calc
                lastPose.x, lastPose.y, lastPose.rot);
        }
        #endif

        // fmt::print("\tESTIMATE: {}", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start));

        // co_await apriltagPublisher->publish(std::move(poses), ts);
        // fmt::print("\tPUBLISH: {}", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start));
        robotTracking.Update(res);
        auto pose = robotTracking.GetRobotPose();
        fmt::println("{}, {} r:{}", pose.x, pose.y, pose.rot);
        #ifdef DEBUG
        fmt::print("\tTotal Time: {}", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start));
        fmt::println("");
        
        #endif
        #ifdef GUI
        // cv::waitKey(1);
        #endif
        cv::waitKey();
        break;
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
    std::terminate(); 
    co_return 0; 
}
