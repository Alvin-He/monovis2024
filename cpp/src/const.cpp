#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include "apriltag/apriltag.hpp"
#include <boost/chrono.hpp>
// support for time unit postfix, eg: 10ms 10min 10 day 
using namespace std::literals::chrono_literals;

namespace K {

auto APRILTAG_LOOP_UPDATE_INTERVAL = boost::chrono::milliseconds(70); 
int NT_UPDATES_PER_SECOND = 11;

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
    }
};

};