#include "global.cpp"
#include "const.cpp"
#include "helpers.cpp"
#include <csignal>
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/program_options.hpp>
#include "apriltag/apriltag.hpp"
#include "network/publishers.cpp"
#include "network/network_time.cpp"

#include "ntcore/networktables/NetworkTableInstance.h"

namespace cobalt = boost::cobalt;
namespace asio = boost::asio; 
namespace PO = boost::program_options;

cobalt::main co_main(int argc, char* argv[]) {  
    // cli argument parsing
    std::string UUID;
    std::string ntRioIP;
    uint ntRioPort;
    std::string redisIP; 
    uint redisPort; 

    PO::options_description cliOptions("Command Line Arguments");
    cliOptions.add_options()
        ("help", "show help message")
        ("uuid,u", 
            PO::value<std::string>(&UUID)
            ->default_value("unnamed"), 
            "service uinque resource ID/name")
        ("nt-ip,n", 
            PO::value<std::string>(&ntRioIP)
            ->default_value("127.0.0.1"), 
            "networktables server ip address")
        ("nt-port", 
            PO::value<uint>(&ntRioPort)
            ->default_value(5810), 
            "networktables server ip address")
        ("redis-ip,r", 
            PO::value<std::string>(&redisIP)
            ->default_value("127.0.0.1"), 
            "monovis redis server ip address")
        ("redis-port", 
            PO::value<uint>(&redisPort)
            ->default_value(6379), 
            "monovis redis server port")
    ;
    PO::variables_map cliArgMap; 
    PO::store(PO::command_line_parser(argc, argv).options(cliOptions).run(), cliArgMap); 
    PO::notify(cliArgMap); 

    if (cliArgMap.contains("help")) {
        std::cout << cliOptions << std::endl;
        co_return 1; 
    } 

    // init redis 
    co_await RedisDB::init(redisIP, std::to_string(redisPort), UUID); 
    co_await RedisDB::ping();

    // init network tables
    nt::NetworkTableInstance ntRio = nt::NetworkTableInstance::Create();
    ntRio.StartClient4("monovis-pose-" + UUID);
    ntRio.SetServer(ntRioIP.c_str(), ntRioPort); 
    auto ntRioRoot = ntRio.GetTable("SmartDashboard");
    
    Publishers::RobotPosePublisher roboPosPublisher {UUID, ntRioRoot}; 

    Apriltag::World::World robotTracking; 
       
    // signal handlers
    std::signal(SIGINT, [](int i){ f_exit = true; }); 
    std::signal(SIGTERM, [](int i){ f_exit = true; }); 
    std::signal(SIGABRT, [](int i){ f_exit = true; }); 

    // main program loop
    // try {
    fmt::println("starting main program loop");
    boost::asio::steady_timer timer {co_await cobalt::this_coro::executor};
    while (!f_exit)
    {   
        timer.expires_from_now(K::POSE_LOOP_UPDATE_INTERVAL); 

        auto [timestamps, validPackets] = co_await Publishers::Internal::ApriltagPosePublisher::receive();
        std::vector<Apriltag::World::RobotPose> poses;
        for (auto& packet : validPackets) {
            poses.push_back(std::move(packet.pose));
        }

        auto avgTs = h::average(timestamps); 

        robotTracking.Update(poses);
        auto robotGlobalPose = robotTracking.GetRobotPose();
        roboPosPublisher(robotGlobalPose, avgTs); 

        #ifdef DEBUG 
        fmt::println("UPDATE: X: {}, Y: {}, R: {}", robotGlobalPose.x, robotGlobalPose.y, robotGlobalPose.rot);
        #endif

        co_await timer.async_wait(cobalt::use_op);
    } 
    // } catch (...) {
    //     std::printf("Exception!"); 
    // };
    
    // exit handling 
    fmt::println("Exiting..."); 
    ntRio.StopClient(); 

    std::terminate();
    co_return 0; 
}
