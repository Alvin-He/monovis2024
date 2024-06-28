#pragma once
// absoulote position tracking
#include "helpers.cpp"
#include "global.cpp"
#include "apriltag/common.cpp"
#include "apriltag/estimator.cpp"

#include <fmt/include/fmt/ranges.h>
#include <boost/math/distributions.hpp>
namespace Apriltag::World {

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

// Tracking for everything known in this world
class World {
    public:
    // Typedefs
    typedef std::vector<RobotPose> CordinateList; 
    struct Group {
        RobotPose cord; 
        size_t count; // used to determine the best cord
    }; 
    // Typedefs

    // Solvers
    static std::vector<double> CamRelativeToAbsoulote(double cx, double cy, double tx, double ty, double tYaw) {
        double theta = h::NormalizeAngle(180.0 - tYaw); 
        std::vector<double> cords = h::rotatePoint(cx, cy, theta); 
        return std::vector<double> {tx - cords[0], ty - cords[1]}; 
    } // CamRelativeToAbsoulote

    static std::vector<Group> GroupCords(const CordinateList& cords, double limitRadiCM = 10) {
        std::vector<Group> groups; 

        groups.push_back(Group {.cord = cords[0], .count = 1}); // initialize with first cordinate 

        size_t totalNumOfCords = cords.size(); 
        for (size_t i = 0; i < totalNumOfCords; i++) {
            RobotPose current = cords[i]; 

            // calculate distance from current cord to all know groups 
            vector_d distances; 
            for (Group g : groups) {
                distances.push_back(std::sqrt(std::pow(g.cord.x - current.x, 2) + std::pow(g.cord.y - current.y, 2))); 
            }

            // assign current cord to a group or make a new group based on distance
            bool isNewGroup = true; 
            size_t groupIndexToJoin = 0;

            size_t totalNumOfGroups = groups.size(); // distances have same size as groups
            for (size_t i = 0; i < totalNumOfGroups; i++) {
                double d = distances[i]; 
                if (d > limitRadiCM) {
                    groupIndexToJoin = i; 
                    isNewGroup = false; 
                }
            }

            if (isNewGroup) { // make new group if needed
                groups.push_back(Group {.cord = current, .count = 1}); 
                continue; 
            }

            // join the group if matched 
            Group g = groups[groupIndexToJoin]; 
            g.cord.x = (g.cord.x + current.x)/2; // average the old cords with the new ones
            g.cord.y = (g.cord.y + current.y)/2;
            // g.cord.rot = (g.cord.rot + current.rot)/2; // yaw calculations are done seperately
            g.count += 1; // increment count 
        }
        return groups; 
    }

    static RobotPose FindBestCord(const std::vector<Group>& groups) {
        size_t bestResIndex = 0; 
        size_t lastBestScore = 0;
        size_t numOfGroups = groups.size();  
        for (size_t i = 0; i < numOfGroups; i++) {
            Group g = groups[i]; 
            if (g.count > lastBestScore) {
                bestResIndex = i; 
                lastBestScore = g.count; 
            } // otherwise skip
        }
        return groups[bestResIndex].cord; 
    }
    
    static double FindBestYaw(const vector_d& yaws) {
        return h::average(h::reject_outliers_2(yaws));
    }
    // Solvers

    // Methods
    World(WorldInfo worldInfo) 
    : m_worldInfo(worldInfo)
    {} // World

    void Update(const Apriltag::AllEstimationResults& results) {
        if (results.size()<1) return; 

        CordinateList allPossibleCords; 
        vector_d allPossibleYaw; 

        for (Apriltag::EstimationResult res : results) {
            TagLocation tag = m_worldInfo.tags[res.id];

            double yaw; 
            
            yaw = -(*res.camToTagRvec[1]); 
            yaw += res.cameraInfo.camToRobotPos[3] + tag.yaw; 
            yaw = 180.0 - yaw; 
            yaw = h::NormalizeAngle(yaw);
            allPossibleYaw.push_back(yaw); 

            double camX = *res.camToTagTvec[2] + res.cameraInfo.camToRobotPos[0]; // camera parallel/z is same as world x for horzontally mounted camera
            double camY = *res.camToTagTvec[0] + res.cameraInfo.camToRobotPos[1]; // camera through/x is same as world y

            std::vector<double> robotCords = CamRelativeToAbsoulote(camX, camY, tag.x, tag.y, tag.yaw); 

            
            fmt::println("yaw: {}", (double)*res.camToTagRvec[1]);
            fmt::println("distance: {}", std::sqrt(std::pow(robotCords[0], 2) + std::pow(robotCords[1], 2))); 
            allPossibleCords.push_back(RobotPose{
                .x = robotCords[0],
                .y = robotCords[1], 
                .rot = yaw
            }); 
        }

        // find most accurate robot cord
        std::vector<Group> cordGroups = GroupCords(allPossibleCords); 
        RobotPose bestPose = FindBestCord(cordGroups); 
        bestPose.rot = FindBestYaw(allPossibleYaw); 

        m_lastRobotPose = bestPose;
    } // Update

    RobotPose GetRobotPose() {
        return m_lastRobotPose; 
    }
    
    private:
        WorldInfo m_worldInfo; 

        RobotPose m_lastRobotPose = {.x = 0, .y=0, .rot = 0}; 

}; // class World

} // namespace Apriltag::World