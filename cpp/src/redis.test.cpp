
#include "common.cpp"
#include "const.cpp"

#include "network/redis.cpp"

#include <boost/asio/steady_timer.hpp>

cobalt::main co_main(int argc, char* argv[]) {

    co_await RedisDB::init();
    
    

    const int scanDataSectionOffset = 3;

    std::string index = "0";
    do {    
        redis::request req; 
        req.push("SCAN", index, "MATCH", "Apriltag:*");

        auto res = *co_await RedisDB::send(req); 

        index = std::move(res[1].value); // blob-string, SCAN index

        size_t currentResponseSize = res[2].aggregate_size;

        for (int i = 0; i < currentResponseSize; i++) {
            fmt::println("Match {}", res[i + scanDataSectionOffset].value);
        }

        fmt::println("index now at {}", index);
    } while (index != "0");
    
    std::terminate();
}