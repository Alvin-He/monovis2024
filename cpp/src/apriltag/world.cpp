#pragma once
// absoulote position tracking
#include "helpers.cpp"
#include "const.cpp"
#include "global.cpp"
#include "apriltag/common.cpp"
#include "apriltag/estimator.cpp"
#include "apriltag/worldDef.cpp"

#include <fmt/include/fmt/ranges.h>
#include <boost/math/distributions.hpp>
#include <cmath>
#include <vector>
namespace Apriltag::World {

// Solvers
static std::array<double, 2> CamRelativeToAbsoulote(double cx, double cy, double tx, double ty, double tYaw) {
    double theta = h::NormalizeAngle(180.0 - tYaw); 
    auto cords = h::rotatePoint(cx, cy, theta); 
    return std::array<double, 2> {tx - cords[0], ty - cords[1]}; 
} // CamRelativeToAbsoulote

static std::vector<Group> GroupCords(const CordinateList& cords, double limitRadiCM = 10) {    
    std::vector<Group> groups; 

    groups.emplace_back(cords[0], 1); // initialize with first cordinate 
    int totalNumOfCords = cords.size(); 
    for (int i = 0; i < totalNumOfCords; i++) {
        int currentNumOfGroups = groups.size();
        RobotPose current = cords[i]; 

        // calculate distance from current cord to all know groups 
        std::vector<double> distances;
        distances.reserve(currentNumOfGroups);  
        for (Group g : groups) {
            // find distance of tag to each group with Pathogram Therom
            distances.emplace_back(std::sqrt(std::pow(g.cord.x - current.x, 2) + std::pow(g.cord.y - current.y, 2))); 
        }

        // assign current cord to a group or make a new group based on distance
        bool isNewGroup = true; 
        // iterate through all distances to each group, check if distance < limitRadiCM
        for (int i = 0; i < currentNumOfGroups; i++) {
            double d = distances[i]; 
            if (d < limitRadiCM) { // the point is close enough to the group
                // join the group if matched 
                Group& g = groups[i]; 
                g.cord.x = (g.cord.x + current.x)/2; // average the old cords with the new ones
                g.cord.y = (g.cord.y + current.y)/2;
                // g.cord.rot = (g.cord.rot + current.rot)/2; // yaw calculations are done seperately
                g.count += 1; // increment count

                isNewGroup = false; //unmark makeNewGroup
                break;
            }
        }

        if (isNewGroup) { // make new group if needed
            groups.emplace_back(current, 1); 
            continue; 
        }
    }
    return std::move(groups); 
}

static RobotPose FindBestCord(const std::vector<Group>& groups) {
    int bestResIndex = 0; 
    int lastBestScore = 0;
    int numOfGroups = groups.size();  
    for (int i = 0; i < numOfGroups; i++) {
        int numTagsInGroup = groups[i].count; 
        if (numTagsInGroup > lastBestScore) {
            bestResIndex = i; 
            lastBestScore = numTagsInGroup; 
        } // otherwise skip
    }
    return groups[bestResIndex].cord; 
}

static double FindBestYaw(const CordinateList& cords) {
    std::vector<double> yaws;
    yaws.reserve(cords.size());
    for (const auto& cord : cords) {
        yaws.emplace_back(cord.rot); 
    }
    return h::average(h::reject_outliers_2(yaws));
}

static RobotPose RobotPoseFromEstimationResult(const Apriltag::EstimationResult& res) {
    //            fmt::println("tvec: {}", res.camToTagTvec); 
    //            fmt::println("rvev: {}", res.camToTagRvec); 

    TagLocation tag = K::FIELD.tags[res.id]; 

    double yaw; 
    
    yaw = -(*res.camToTagRvec[1]); 
    yaw += res.cameraInfo->camToRobotPos[3] + tag.yaw; 
    yaw = 180.0 - yaw; 
    yaw = h::NormalizeAngle(yaw);

    double camX = *res.camToTagTvec[2] + res.cameraInfo->camToRobotPos[0]; // camera parallel/z is same as world x for horzontally mounted camera
    double camY = *res.camToTagTvec[0] + res.cameraInfo->camToRobotPos[1]; // camera through/x is same as world y

    std::array<double, 2> robotCords = CamRelativeToAbsoulote(camX, camY, tag.x, tag.y, tag.yaw); 

    return RobotPose {.x = robotCords[0], .y = robotCords[1], .rot = yaw};
}
// Solvers

// Tracking for everything known in this world
class World {
    public:
    // Methods
    World() = default;

    void Update(const Apriltag::AllEstimationResults& results) {
        CordinateList allPossibleCords; 
        for (const Apriltag::EstimationResult& res : results) {
            allPossibleCords.push_back(RobotPoseFromEstimationResult(res)); 
        }
        Update(std::move(allPossibleCords)); 
    }

    void Update(const CordinateList& poses) {
        if (poses.size()<1) return; 

        // find most accurate robot cord
        std::vector<Group> cordGroups = GroupCords(poses); 
        RobotPose bestPose = FindBestCord(cordGroups); 
        bestPose.rot = FindBestYaw(poses); 

        m_lastRobotPose = bestPose;
    } // Update

    RobotPose GetRobotPose() {
        return m_lastRobotPose; 
    }
    
    private:
        RobotPose m_lastRobotPose = {.x = 0, .y=0, .rot = 0}; 

}; // class World

} // namespace Apriltag::World
