
#include "global.cpp"
#include "const.cpp"
#include "helpers.cpp"
#include <csignal>
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/program_options.hpp>
#include <string>
#include "camera/camera.cpp"
#include "apriltag/apriltag.hpp"
#include "network/publishers.cpp"
#include "network/network_time.cpp"
#include "network/redis.cpp"

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 
namespace PO = boost::program_options;

cobalt::main co_main(int argc, char* argv[]) {  
    // cli argument parsing
    std::string UUID;
    std::string redisIP; 
    uint redisPort; 
    int cameraID; 
    std::string cameraCalibrationFilePath;

    PO::options_description cliOptions("Command Line Arguments");
    cliOptions.add_options()
        ("help", "show help message")
        ("uuid,u", 
            PO::value<std::string>(&UUID)
            ->default_value("monovis-unnamed"), 
            "service uinque resource ID/name")
        ("redis-ip", 
            PO::value<std::string>(&redisIP)
            ->default_value("127.0.0.1"), 
            "monovis internal redis server ip address")
        ("redis-port", 
            PO::value<uint>(&redisPort)
            ->default_value(6379), 
            "monovis internal redis server port")
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

    // init RedisDB access
    co_await RedisDB::init(redisIP, std::to_string(redisPort), UUID); 
    co_await RedisDB::ping();

    // bootstrap camera
    cv::VideoCapture cap{cameraID}; 

    // static to ensure cameraData always exists 
    static Camera::CameraData cameraData = Camera::LoadCalibDataFromXML(cameraCalibrationFilePath);
    cameraData.id = cameraID; 
    Camera::AdjustCameraDataToForFrameSize(cameraData, cap, K::PROC_FRAME_SIZE);
    Camera::FrameGenerator cameraReader {cap};
    std::shared_ptr<Camera::CameraData> s_cameraData (&cameraData); 

    fmt::println("camera {} initiated", cameraID);

    // construct estimator
    Apriltag::Estimator estimator {s_cameraData, APRILTAG_DETECTOR_PARAMS};

    // construct publisher
    Publishers::Internal::ApriltagPosePublisher apriltagPublisher {UUID}; 

    // signal handlers
    std::signal(SIGINT, [](int i){ f_exit = true; }); 
    std::signal(SIGTERM, [](int i){ f_exit = true; }); 
    std::signal(SIGABRT, [](int i){ f_exit = true; }); 

    // main program loop
    // try {
    fmt::println("starting main program loop");
    while (!f_exit)
    {   
        #ifdef DEBUG
        boost::timer::cpu_timer timer; 
        timer.start(); 
        #endif 

        cv::Mat frame = co_await cameraReader.PromiseRead();
        auto ts = NetworkTime::Now();
        frame = co_await Camera::PromiseResize(frame, K::PROC_FRAME_SIZE);
        // fmt::println("IO & PREPROC Time: {}ms", timer.elapsed().wall/1000000);

        Apriltag::AllEstimationResults res = co_await estimator.PromiseDetect(frame); 

        std::vector<Publishers::Internal::ApriltagPose> poses;
        for(auto& esti : res) {
            poses.push_back(Publishers::Internal::ApriltagPose {
                .id = esti.id, 
                .pose = Apriltag::World::RobotPoseFromEstimationResult(std::move(esti))
            });
        }
        co_await apriltagPublisher(std::move(poses), ts);

        #ifdef DEBUG
        // fmt::println("Cycle Time: {}ms", timer.elapsed().wall/1000000.0);
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

    cap.release(); 
    #ifdef GUI
    cv::destroyAllWindows(); 
    #endif
    std::terminate(); 
    co_return 0; 
}
