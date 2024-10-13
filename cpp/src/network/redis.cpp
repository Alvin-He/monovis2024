#pragma once 

#include "common.cpp"
#include "global.cpp"
#include "const.cpp"
#include <boost/redis.hpp>
#include <boost/redis/src.hpp>
#include <networktables/Topic.h>

namespace redis = boost::redis;

namespace RedisDB {
    std::shared_ptr<redis::config> m_config;
    std::unique_ptr<redis::connection> m_connection = nullptr; 
    std::unique_ptr<cobalt::promise<void>> m_NQSendLastPromise;
    int failsendcount = 0;

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
            .resolve_timeout = 100ms, 
            .connect_timeout = 100ms,
            .health_check_interval = 3s,
            .reconnect_wait_interval = 20ms,       
        });


        m_connection = std::make_unique<redis::connection>(co_await cobalt::this_coro::executor); 
        // run detached server loop
        [&] () -> cobalt::detached { co_await m_connection->async_run(*m_config, redis::logger(redis::logger::level::debug), cobalt::use_op); }(); 

        
        fmt::println("redis initiated");

        // initialize to a no-op immediate completed promise
        m_NQSendLastPromise = std::make_unique<cobalt::promise<void>>([]() -> cobalt::promise<void>{co_return;}());
        co_return;
    };

    cobalt::promise<redis::generic_response> send(redis::request& req) {
        req.get_config().cancel_if_not_connected = true;

        redis::generic_response res; 
        try { co_await m_connection->async_exec(req, res, cobalt::use_op); }
        catch(...) {
            fmt::println("async_exec failed!, #{}", ++failsendcount); 
        }
        co_return res; 
    }

    template <typename T>
    cobalt::promise<redis::response<T>> send(redis::request& req) {
        req.get_config().cancel_if_not_connected = true;

        redis::response<T> res; 
        try { co_await m_connection->async_exec(req, res, cobalt::use_op); }
        catch(...) {
            fmt::println("async_exec failed!, #{}", ++failsendcount); 
        }
        co_return res; 
    }

    cobalt::promise<void> sendDiscard(redis::request& req) {
        co_await send(req); 
        co_return;
    }

    cobalt::promise<void> noQueueSendDiscard(redis::request& req) {
        if (m_NQSendLastPromise->ready()) { 
            m_NQSendLastPromise = std::make_unique<cobalt::promise<void>>(std::move(sendDiscard(req)));
        } // else wait out the promise
        co_await *m_NQSendLastPromise;
        // then fire the request 
        m_NQSendLastPromise = std::make_unique<cobalt::promise<void>>(std::move(sendDiscard(req)));
        co_return;
    }

    cobalt::promise<void> ping() {
        redis::request req; 
        req.push("PING", m_config->clientname + " PING");

        redis::response<std::string> res = co_await send<std::string>(req); 

        fmt::println("PING: {}", std::get<0>(res).value()); 
        co_return; 
    };

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
