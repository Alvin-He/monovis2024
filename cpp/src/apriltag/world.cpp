#pragma once
// absoulote position tracking
#include "helpers.cpp"
#include "global.cpp"
#include "apriltag/common.cpp"
#include "apriltag/estimator.cpp"

#include <fmt/include/fmt/ranges.h>

namespace Apriltag::World {

// Storage container for individual Tag info
struct TagLocation {
    int id = 0; // int
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

// Tracking for everything known in this world
class World {
    public:
    static std::vector<double> CamRelativeToAbsoulote(double cx, double cy, double tx, double ty, double tYaw) {
        double theta = NormalizeAngle(180.0 - tYaw); 
        std::vector<double> cords = rotatePoint(cx, cy, theta); 
        return std::vector<double> {tx - cords[0], ty - cords[1]}; 
    } // CamRelativeToAbsoulote

    typedef std::vector<RobotPose> CordinateList; 

    World(WorldInfo worldInfo) 
    : m_worldInfo(worldInfo)
    {} // World

    void Update(Apriltag::AllEstimationResults results) {
        if (results.size()<1) return; 

        CordinateList allPossibleCords; 
        
        for (Apriltag::EstimationResult res : results) {
            TagLocation tag = m_worldInfo.tags[res.id];

            double yaw; 
            yaw = -(*res.camToTagRvec[1]); 
            yaw += res.cameraInfo.camToRobotPos[3] + tag.yaw; 
            yaw = 180 - yaw; 
            yaw = NormalizeAngle(yaw);

            double camX = *res.camToTagTvec[2] + res.cameraInfo.camToRobotPos[0]; // camera parallel/z is same as world x for horzontally mounted camera
            double camY = *res.camToTagTvec[0] + res.cameraInfo.camToRobotPos[1]; // camera through/x is same as world y

            std::vector<double> robotCords = CamRelativeToAbsoulote(camX, camY, tag.x, tag.y, tag.yaw); 
                 
            fmt::println("cord: {}", robotCords); 
            allPossibleCords.push_back(RobotPose{
                .x = robotCords[0],
                .y = robotCords[0], 
                .rot = yaw
            }); 
        }
    } // Update


    private:
        WorldInfo m_worldInfo; 

        RobotPose m_lastRobotPose = {.x = 0, .y=0, .rot = 0}; 

}; // class World

} // namespace Apriltag::World