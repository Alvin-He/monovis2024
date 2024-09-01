#pragma once
#include <vector>

// Types and container definations used in world.cpp

namespace Apriltag::World
{        
    // Storage container for individual Tag info
    struct TagLocation {
        long id = 0; // int
        double x = 0; // cm
        double y = 0; // cm
        double z = 0; // cm 
        double yaw = 0; // deg
    }; // struct TagLocation

    // Tag locations ordered by their ID
    typedef std::vector<TagLocation> AllTagsInfo; 

    // Storage container for World info
    struct WorldInfo {
        double xTot; 
        double yTot;
        AllTagsInfo tags;
    }; // Struct WorldInfo

    struct RobotPose {
        double x; 
        double y; 
        double rot;
    }; 

    struct Group {
        RobotPose cord; 
        int count; // used to determine the best cord
    }; 

    // Typedefs
    typedef std::vector<RobotPose> CordinateList; 

} // namespace Apriltag::World
