#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include "world/TypeDefs.cpp"
#include <boost/chrono.hpp>
// support for time unit postfix, eg: 10ms 10min 10 day 
using namespace std::literals::chrono_literals;

namespace K {

auto APRILTAG_LOOP_UPDATE_INTERVAL = boost::chrono::milliseconds(70); 
auto APRILTAG_POSE_LIFE_DURATION = 1s;
int NT_UPDATES_PER_SECOND = 11;

uint APRILTAG_CACHE_COUNT = 50;
auto POSE_TIME_CONSIDERATION = boost::chrono::milliseconds(100).count(); 
auto POSE_LOOP_UPDATE_INTERVAL = 90ms; 

cv::Size2d PROC_FRAME_SIZE (640, 480); 
};
