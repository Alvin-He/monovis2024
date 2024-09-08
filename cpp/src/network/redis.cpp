#pragma once 

#include "common.cpp"
#include "global.cpp"
#include "const.cpp"
#include <boost/redis.hpp>
#include <boost/redis/src.hpp>

namespace redis = boost::redis;

namespace RedisDB {
    std::shared_ptr<redis::config> m_config;
    std::unique_ptr<redis::connection> m_connection = nullptr; 

    cobalt::promise<void> init(std::string ip = "127.0.0.1", std::string port = "6379", std::string name = "monovis-redis-unnamed") {
        if (m_connection) {
            fmt::println("RedisDB already initiated, skipping this init call.");
            co_return; 
        }

        m_config = std::make_shared<redis::config> (redis::config {
            .addr = redis::address {.host = ip, .port = port}, 
            .clientname = name, 
            .health_check_id = name,
            .log_prefix = name, 
            .health_check_interval = 0s
        });

        m_connection = std::make_unique<redis::connection>(co_await cobalt::this_coro::executor); 
        // run detached server loop
        [&] () -> cobalt::detached { co_await m_connection->async_run(*m_config, {}, cobalt::use_op); }(); 
        fmt::println("redis connected");
        co_return;
    };

    cobalt::promise<void> ping() {
        redis::request req; 
        req.push("PING", m_config->clientname + " PING");

        redis::response<std::string> res; 
        co_await m_connection->async_exec(req, res, cobalt::use_op);

        fmt::println("PING: {}", std::get<0>(res).value()); 
        co_return; 
    };

    cobalt::promise<redis::generic_response> send(redis::request req) {
        redis::generic_response res; 
        co_await m_connection->async_exec(req, res, cobalt::use_op); 
        co_return res; 
    }

    constexpr int scanDataSectionOffset = 3;
    // scans database for keys matching pattern, returning all matched keys
    cobalt::promise<std::vector<std::string>> scan(std::string pattern) {
        std::vector<std::string> retBuf; 

        std::string index = "0";
        do {    
            //construct scan request
            redis::request req; 
            req.push("SCAN", index, "MATCH", pattern);

            auto res = *co_await RedisDB::send(req);

            // process response
            index = std::move(res[1].value); // blob-string, SCAN index

            size_t currentResponseSize = res[2].aggregate_size; // array, no value, but aggergate reflects data section size

            // parse data section
            for (int i = 0; i < currentResponseSize; i++) {
                retBuf.push_back(std::move(res[i + scanDataSectionOffset].value));
            }

        } while (index != "0");

        co_return retBuf;
    }

}
