// Global configurations, variables and constants 
#pragma once


// #define DEBUG true
// #define GUI true
#define HAS_CUDA true
#define BOOST_THREAD_VERSION 5

#if __has_include("dev_overwrite_global.cpp")
    #include "dev_overwrite_global.cpp"
#endif

constexpr double APRILTAG_BLOCK_SIZE_cm = 2.0;  