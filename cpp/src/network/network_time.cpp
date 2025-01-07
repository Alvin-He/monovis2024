#pragma once

#include "common.cpp"

#include "ntcore/networktables/NetworkTableInstance.h"

#include <boost/asio/steady_timer.hpp>

namespace NetworkTime {
    std::atomic<int64_t> serverTimeOffset_us = nt::NetworkTableInstance::GetDefault().GetServerTimeOffset().value_or(0);

    bool ___is_StartPeriodicLatencyUpdate_launched = false; 
    // Update loop to periodically update the local cache of the server time offset
    void StartPeriodicLatencyUpdate() {
        if (___is_StartPeriodicLatencyUpdate_launched) throw std::logic_error("StartPeriodicLatencyUpdate has already been started."); 
        ___is_StartPeriodicLatencyUpdate_launched = true;

        nt::NetworkTableInstance::GetDefault().AddTimeSyncListener(true, [](const nt::Event& e) {
            NetworkTime::serverTimeOffset_us = e.GetTimeSyncEventData()->serverTimeOffset;
        }); 
    };

    // returns the current server time
    int64_t Now() {
        return nt::Now() + NetworkTime::serverTimeOffset_us;
    }

}