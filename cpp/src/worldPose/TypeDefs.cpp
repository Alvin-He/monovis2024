#pragma once
#include <vector>

// Types and container definations used in world.cpp

namespace WorldPose
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

    struct Pos2D {
        double x = 0; 
        double y = 0; 
        double rot = 0;
    }; 

    struct Pos2DwTag { 
        double x = 0; 
        double y = 0; 
        double rot = 0;
        int id = 0; 

        operator Pos2D() const {return Pos2D(x, y, rot);}
    };

    struct Group {
        Pos2D cord; 
        int count; // used to determine the best cord
    }; 

} // namespace Apriltag::World
