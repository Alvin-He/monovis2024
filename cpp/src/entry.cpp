
#include "global.cpp"
#include "helpers.cpp"
#include <csignal>
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/program_options.hpp>
#include "camera/camera.cpp"
#include "apriltag/apriltag.hpp"
#include "network/publishers.cpp"

#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/NetworkTableInstance.h"

#include "conf/parser.cpp"
#include "conf/serviceConfigDef.cpp"
#include "conf/cameraConfigDef.cpp"

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

// maintaines references to states that's needed to run various tasks
struct AppState { // resources should be std::moved into here
    size_t numCameras = 0;
    std::vector<Conf::CameraConfig> config; 
    std::vector<cv::VideoCapture> cap; 
    std::vector<cobalt::generator<cv::Mat>> cameraReader; 
    std::vector<Camera::CameraData> cameraData; 
    std::vector<Apriltag::Estimator> estimator;

    // caches
    std::unique_ptr<cv::Mat> frameCache = nullptr; 

};

bool isFlagExit = false; 
cobalt::main co_main(int argc, char* argv[]) {  

    AppState appState {}; 

    // cli argument parsing
    std::string configFilePath; 

    PO::options_description cliOptions("Command Line Arguments");
    cliOptions.add_options()
        ("help", "show help message")
        ("config,c", 
            PO::value<std::string>(&configFilePath)
            ->default_value("config.toml"), 
            "path to service config file")
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
        appState.config.push_back(Conf::ParseCameraConfig(Conf::LoadToml(file))); 
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
    auto appNetworkTable = ntInst.GetTable("SmartDashboard")->GetSubTable("Vision");
    auto robotPosTable = appNetworkTable->GetSubTable("RobotPos"); 
    auto apriltagGroupTable = appNetworkTable->GetSubTable("Apriltags"); 

    auto robotPosePublisher = Publishers::RobotPosePublisher(robotPosTable); 

    Apriltag::World::World robotTracking {FIELD}; 
    
    for (const Conf::CameraConfig& camConfig : appState.config) {
        // start camera streams
        cv::VideoCapture cap{camConfig.Camera_id}; 

        Camera::CameraData cameraData = Camera::LoadCalibDataFromXML(camConfig.Camera_calibrationFile);
        cameraData.id = camConfig.Camera_id;
        Camera::AdjustCameraDataToForFrameSize(cameraData, cap, PROC_FRAME_SIZE);

        cobalt::generator<cv::Mat> cameraReader = Camera::Reader(cap);
        Apriltag::Estimator estimator {cameraData, APRILTAG_DETECTOR_PARAMS}; 

        appState.cameraData.push_back(std::move(cameraData)); 
        appState.cameraReader.push_back(std::move(cameraReader)); 
        appState.cap.push_back(std::move(cap)); 
        appState.estimator.push_back(std::move(estimator)); 
        appState.numCameras += 1; 
    }
    
    // signal handlers
    std::signal(SIGINT, [](int i){ isFlagExit = true; }); 
    std::signal(SIGTERM, [](int i){ isFlagExit = true; }); 
    std::signal(SIGABRT, [](int i){ isFlagExit = true; }); 

    int gaussianBlurKernal = 5; 
    double gaussianBllueStdev = 0.8;
    #ifdef GUI
    std::string testWindow = "test"; 
    cv::namedWindow(testWindow);

    cv::createTrackbar("Gaussian Blur Kernal", testWindow, nullptr, 10, [](int pos, void* ori) { 
        int* val = static_cast<int*>(ori); 
        if (pos < 3) pos = 3; 
        *val = ((pos % 2) == 0) ? (pos + 1) : pos;  
    }, &gaussianBlurKernal); 
    cv::createTrackbar("Gaussian Blur Stddev", testWindow, nullptr, 30, [](int pos, void* ori) { 
        double* val = static_cast<double*>(ori); 
        pos /= 10; 
        *val = pos;
    }, &gaussianBllueStdev); 
    
    #endif 

    // main program loop
    // try {
    
    int64_t serverTimeOffset = ntInst.GetServerTimeOffset().value_or(0); 
    size_t nThreads = std::thread::hardware_concurrency();
    
    // only create more threads if there are more than 2 threads (no benifit if less than 2, main program uses a thread and networking + the OS will need the other core)
    std::unique_ptr<asio::thread_pool> threadPool = nullptr; 
    if (nThreads > 2) {
        if (nThreads > appState.numCameras) nThreads = appState.numCameras; // no use creating more threads than there are cameras
        threadPool = std::make_unique<asio::thread_pool>(nThreads);
        fmt::println("threads: {}", nThreads);
    }; 

    while (!isFlagExit)
    {   
        auto allCamFrames = co_await cobalt::join(appState.cameraReader); // std::vector<cv::Mat> with pmr as allocator

        // timestamp immediately after we got the frames so lag is minminzed 
        int64_t frameTimeStamp = nt::Now() + serverTimeOffset;  
        
        for (auto&& frame : allCamFrames) { // could put all the CudaResize calls into a cobalt join call to improve performance  
            frame = co_await Camera::CudaResize(frame, PROC_FRAME_SIZE);
        } 
        
        #ifdef GUI
        for (size_t i = 0; i < appState.numCameras; i++) cv::imshow("camera " + std::to_string(appState.cameraData[i].id) , allCamFrames[i]);
        #endif
        boost::timer::cpu_timer timer;         
        timer.start(); 

        Apriltag::AllEstimationResults res; 

        if (threadPool == nullptr) { // single thread mode
            for (size_t i = 0; i < appState.numCameras; i++) {
                auto procRes = co_await appState.estimator[i].PromiseDetect(allCamFrames[i]); 
                res.insert(res.end(), procRes.begin(), procRes.end()); 
            }

        } else { // a thread pool exist
            std::vector<std::future<Apriltag::AllEstimationResults>> allEstimateTasks;

            for (size_t i = 0; i < appState.numCameras; i++) {
                std::future<Apriltag::AllEstimationResults> estimateTask = cobalt::spawn(
                    asio::make_strand(threadPool->get_executor()), 
                    appState.estimator[i].TaskDetect(
                        allCamFrames[i]
                    ),
                    asio::use_future
                );
                allEstimateTasks.push_back(std::move(estimateTask)); 
            }


            for (auto& task : allEstimateTasks) {
                auto procRes = task.get(); 
                res.insert(res.end(), procRes.begin(), procRes.end()); 
            } // flatten the vector<vector<EstimationResult>> into AllEstimationResults
        }


        // Apriltag::AllEstimationResults res = co_await appState.estimator[0].PromiseDetect(allCamFrames[0]); 
        robotTracking.Update(res);
        // fmt::println("time used:{}ms", timer.elapsed().wall/1000000.0); 
        Apriltag::World::RobotPose robotPose = robotTracking.GetRobotPose(); 
        // fmt::println("x: {}, y:{}, r:{}", robotPose.x, robotPose.y, robotPose.rot);
        // fmt::println("distance: {}", std::sqrt(std::pow(robotPose.x, 2) + std::pow(robotPose.y, 2))); 
        co_await robotPosePublisher(Publishers::RobotPosePacket {
            .pose = robotPose, 
            .timestamp = frameTimeStamp
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
    for (auto& cap : appState.cap) cap.release(); 
    #ifdef GUI
    cv::destroyAllWindows(); 
    #endif
    if (threadPool != nullptr) {
        threadPool->stop(); 
        threadPool->join(); 
    }
    co_return 0; 
}
