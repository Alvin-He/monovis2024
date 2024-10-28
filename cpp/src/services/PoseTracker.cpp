#include "fmt/core.h"
#include "global.cpp"
#include "const.cpp"
#include "helpers.cpp"
#include <csignal>
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/program_options.hpp>
#include <memory>
#include "apriltag/apriltag.hpp"
#include "network/RobotPose.cpp"
#include "network/ApriltagPose.cpp"
#include "network/network_time.cpp"

#include "ntcore/networktables/NetworkTableInstance.h"
#include "program_options/value_semantic.hpp"

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
    bool useNTForReceive = false;

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
            "networktables server ip to publish to (and receive from if useNTForReceive)")
        ("nt-port", 
            PO::value<uint>(&ntRioPort)
            ->default_value(5810), 
            "networktables server port to publish to (and receive from if useNTForReceive)")
        ("redis-ip,r", 
            PO::value<std::string>(&redisIP)
            ->default_value("127.0.0.1"), 
            "monovis redis server ip address")
        ("redis-port", 
            PO::value<uint>(&redisPort)
            ->default_value(6379), 
            "monovis redis server port")
        ("useNTForReceive,T", 
            PO::value<bool>(&useNTForReceive)
            ->default_value(false),
            "receive estimation info on NetworkTables instead of redis, only use in single camera set ups! default: false"
        )
    ;
    PO::variables_map cliArgMap; 
    PO::store(PO::command_line_parser(argc, argv).options(cliOptions).run(), cliArgMap); 
    PO::notify(cliArgMap); 

    if (cliArgMap.contains("help")) {
        std::cout << cliOptions << std::endl;
        co_return 1; 
    } 

    // init redis 
    if (!useNTForReceive) {
        co_await RedisDB::init(redisIP, std::to_string(redisPort), UUID); 
        co_await RedisDB::ping();
    }

    // init network tables
    nt::NetworkTableInstance ntRio = nt::NetworkTableInstance::GetDefault();
    ntRio.StartClient4("monovis-pose-" + UUID);
    ntRio.SetServer(ntRioIP.c_str(), ntRioPort); 
    
    std::unique_ptr<Network::RobotPose::Publisher> roboPosPublisher = std::make_unique<Network::RobotPose::NTPublisher>(UUID); 

    std::unique_ptr<Network::ApriltagPose::Receiver> estiInfoReceiver; 
    if (useNTForReceive) estiInfoReceiver = std::make_unique<Network::ApriltagPose::NTReceiver>();
    else estiInfoReceiver = std::make_unique<Network::ApriltagPose::RedisReceiver>(UUID);

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

        auto [timestamps, validPackets] = co_await estiInfoReceiver->receive();

        auto avgTs = h::average(timestamps); 
        robotTracking.Update(validPackets);
        auto robotGlobalPose = robotTracking.GetRobotPose();
        roboPosPublisher->publish(robotGlobalPose, avgTs); 

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
