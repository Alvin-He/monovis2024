// Global configurations, variables and constants 
#pragma once


// #define DEBUG true
// #define GUI true

#if __has_include("dev_overwrite_global.cpp")
    #include "dev_overwrite_global.cpp"
#endif

constexpr double APRILTAG_BLOCK_SIZE_cm = 2.0;  