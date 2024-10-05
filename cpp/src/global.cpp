// Global configurations, variables and constants 
#pragma once
#include <atomic>

// #define DEBUG true
// #define GUI true
#define FAIL_SILENT true

#define BOOST_THREAD_VERSION 5

std::atomic<bool> f_exit = false; 

constexpr double APRILTAG_BLOCK_SIZE_cm = 2.0;  

#if __has_include("opencv4/opencv2/cudawarping.hpp") 
    #define HAS_CUDA true
#endif

#if __has_include("dev_overwrite_global.cpp")
    #include "dev_overwrite_global.cpp"
#endif