#pragma once

#include "common.cpp"

#include "ntcore/networktables/NetworkTableInstance.h"

#include <boost/asio/steady_timer.hpp>

namespace NetworkTime {
    int64_t serverTimeOffset = nt::NetworkTableInstance::GetDefault().GetServerTimeOffset().value_or(0);

    bool ___is_StartPeriodicLatencyUpdate_launched = false; 
    // Update loop to periodically update the local cache of the server time offset
    cobalt::detached StartPeriodicLatencyUpdate(const std::chrono::milliseconds updateInterval = 1000ms) {
        if (___is_StartPeriodicLatencyUpdate_launched) throw std::logic_error("StartPeriodicLatencyUpdate is called already."); 
        ___is_StartPeriodicLatencyUpdate_launched = true;

        boost::asio::steady_timer timer{co_await cobalt::this_coro::executor, updateInterval};
        for(;;) {
            auto offset = nt::NetworkTableInstance::GetDefault().GetServerTimeOffset();
            if (offset.has_value()) {
                NetworkTime::serverTimeOffset = offset.value(); 
            }
            co_await timer.async_wait(cobalt::use_op);
            timer.expires_at(timer.expires_at() + updateInterval);
        }
    };

    // returns the current server time
    // can be used in threads, serverTimeOffset is not procted by a lock, but it donesn't need it
    int64_t Now() {
        return nt::Now() + NetworkTime::serverTimeOffset;
    }

}