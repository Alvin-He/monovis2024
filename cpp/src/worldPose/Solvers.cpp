#pragma once

#include "helpers.cpp"
#include "TypeDefs.cpp"
#include "apriltag/Estimator.ipp"
#include <tuple>
#include <vector>

namespace WorldPose::Solvers { // Solvers
    std::pair<double, double> CamRelativeToAbsoulote(double cx, double cy, double tx, double ty, double tYaw) {
        double theta = h::NormalizeAngle(180.0 - tYaw); 
        auto cords = h::rotatePoint(cx, cy, theta); 
        return {tx - cords[0], ty - cords[1]}; 
    } // CamRelativeToAbsoulote


    Pos2DwTag RobotPoseFromEstimationResult(const Apriltag::EstimationResult& res) {
        //            fmt::println("tvec: {}", res.camToTagTvec); 
        //            fmt::println("rvev: {}", res.camToTagRvec); 

        TagLocation tag = K::FIELD.tags[res.id]; 

        double yaw; 
        
        yaw = -(*res.camToTagRvec[1]); 
        yaw += res.cameraInfo->cameraPos[3] - tag.yaw; 
        yaw = 180.0 - yaw; 
        yaw = h::NormalizeAngle(yaw);

        double camX = *res.camToTagTvec[2] - res.cameraInfo->cameraPos[0]; // camera parallel/z is same as world x for horzontally mounted camera
        double camY = *res.camToTagTvec[0] - res.cameraInfo->cameraPos[1]; // camera through/x is same as world y

        auto robotCords = CamRelativeToAbsoulote(camX, camY, tag.x, tag.y, tag.yaw); 

        return Pos2DwTag {.x = robotCords.first, .y = robotCords.second, .rot = yaw, .id = res.id};
    }

    std::vector<Group> GroupCords(const std::vector<Apriltag::EstimationResult>& cords, double limitRadiCM = 20) {    
        std::vector<Group> groups; 

        // groups.emplace_back(cords[0], 1); // initialize with first cordinate 
        int totalNumOfCords = cords.size(); 
        for (int i = 0; i < totalNumOfCords; i++) {
            int currentNumOfGroups = groups.size();
            const Pos2DwTag& current = RobotPoseFromEstimationResult(cords[i]); 

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
                    g.cord.rot = (g.cord.rot + current.rot)/2; // average yaw
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

    Pos2D FindBestCord(const std::vector<Group>& groups) {
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

    double FindBestYaw(const std::vector<Pos2D>& cords) {
        std::vector<double> yaws;
        yaws.reserve(cords.size());
        for (const auto& cord : cords) {
            yaws.emplace_back(cord.rot); 
        }
        return h::average(h::reject_outliers_2(yaws));
    }
} // Solvers