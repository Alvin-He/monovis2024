#pragma once
#include <vector>
// Types and container definations used in world.cpp

namespace World
{
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
