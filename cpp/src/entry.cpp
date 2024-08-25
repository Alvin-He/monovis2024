
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

#include "async/pipeline.cpp"

#include "tasks.cpp"

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 
namespace PO = boost::program_options;

// maintaines references to states that's needed to run various tasks
struct AppState { // resources should be std::moved into here
    size_t numCameras = 0;

    std::vector<std::shared_ptr<State>> cameraStates; 

    std::vector<Conf::CameraConfig> cameraConfigs; 

    // Long live objects, these are just held for ownership and no one really uses them
    std::vector<cv::VideoCapture> cameraCaps; 
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
    
    for (const Conf::CameraConfig& camConfig : appState.cameraConfigs) {
        // start camera streams
        cv::VideoCapture cap{camConfig.Camera_id}; 

        Camera::CameraData cameraData = Camera::LoadCalibDataFromXML(camConfig.Camera_calibrationFile);
        cameraData.id = camConfig.Camera_id;
        Camera::AdjustCameraDataToForFrameSize(cameraData, cap, K::PROC_FRAME_SIZE);

        Camera::FrameGenerator cameraReader {cap};
        Apriltag::Estimator estimator {cameraData, APRILTAG_DETECTOR_PARAMS}; 

        State cameraState = {
            .config = Conf::CameraConfig(camConfig),
            .cameraData = std::move(cameraData),

            .estimator = std::move(estimator),
            .frameGen = std::move(cameraReader), 
        }; 

        appState.cameraCaps.push_back(std::move(cap)); 
        
        appState.cameraStates.push_back(std::make_shared<State>(std::move(cameraState))); 
        appState.numCameras += 1; 
    }
    
    // signal handlers
    std::signal(SIGINT, [](int i){ isFlagExit = true; }); 
    std::signal(SIGTERM, [](int i){ isFlagExit = true; }); 
    std::signal(SIGABRT, [](int i){ isFlagExit = true; }); 

    // main program loop
    // try {
    
    size_t nThreads = boost::thread::hardware_concurrency();
    if (nThreads == 0) nThreads = 1;

    std::vector<boost::thread> threads; 
    for (auto state : appState.cameraStates) {
        threads.push_back(boost::thread {NormalCameraPipeline, state}); 
    }

    auto updateInterval = (1000ms/K::NT_UPDATES_PER_SECOND);
    boost::asio::steady_timer timer{co_await cobalt::this_coro::executor, updateInterval};
    while (!isFlagExit)
    {   
        co_await timer.async_wait(cobalt::use_op);
        timer.expires_at(timer.expires_at() + updateInterval);

        Apriltag::AllEstimationResults fusedRes; 
        std::vector<int64_t> allTimestamps; 
        for (auto& state : appState.cameraStates) {
            auto estiRes = state->estimationResult; 
            auto frameTs = state->frameTimeStamp;
            if (!estiRes || !frameTs) continue;
            
            if (estiRes->size() < 1) continue;
            
            fusedRes.insert(fusedRes.end(), estiRes->begin(), estiRes->end());
            allTimestamps.push_back(*frameTs);
        }
        int64_t fusedTs = h::average(allTimestamps); 

        robotTracking.Update(fusedRes);
        Apriltag::World::RobotPose robotPose = robotTracking.GetRobotPose(); 
        // fmt::println("x: {}, y:{}, r:{}", robotPose.x, robotPose.y, robotPose.rot);
        // fmt::println("distance: {}", std::sqrt(std::pow(robotPose.x, 2) + std::pow(robotPose.y, 2))); 
        co_await robotPosePublisher(Publishers::RobotPosePacket {
            .pose = robotPose, 
            .timestamp = fusedTs
        }); 

        // for (Apriltag::EstimationResult estimation : fusedRes) {
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
    for (auto& cap : appState.cameraCaps) cap.release(); 
    #ifdef GUI
    cv::destroyAllWindows(); 
    #endif 
    for (auto& thread : threads) thread.interrupt();
    for (auto& thread : threads) {
        bool suscess = thread.try_join_for(boost::chrono::seconds(5));
        if (!suscess) pthread_kill(thread.native_handle(), 9); // send SIGKILL
    } 

    exit(0);
    co_return 0; 
}
