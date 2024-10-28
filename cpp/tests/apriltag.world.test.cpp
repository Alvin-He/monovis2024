#include "common.cpp"
#include "fmt/core.h"
#include "math/tools/assert.hpp"
#include <cassert>
#include "apriltag/apriltag.hpp"


cobalt::main co_main(int argc, char **argv) {
    
    Apriltag::World::CordinateList cords {
        {0, 0, 0},
        {243, 167, 30},
        {210,326, 75},
        {205, 332, 78},
        {197, 340.342, 80},
        {156.748, 132, 270},
        {-300.431, -100.232, 11}, 
        {-50,100, -163},
        {230, -160, 324}
    };

    auto isInExpected = [&] (Apriltag::World::Pos2D pose) {
        return
            197 < pose.x && pose.x < 210 &&
            326 < pose.y && pose.y < 340 &&
            78 < pose.rot && pose.rot < 80;  
    };

    Apriltag::World::World world;
    world.Update(cords); 

    fmt::println("({}, {}) r:{}", world.GetRobotPose().x, world.GetRobotPose().y, world.GetRobotPose().rot); 
    assert(isInExpected(world.GetRobotPose()) && "world update failed to optimize to the expected pose."); 

    co_return 0;
}
