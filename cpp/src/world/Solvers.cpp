#pragma once

#include "helpers.cpp"
#include "TypeDefs.cpp"
#include "apriltag/Estimator.ipp"
#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <utility>
#include <vector>
#include "Field.cpp"


namespace World::Solvers { // Solvers
    std::array<cv::Point3f, 4> m_objectPoints = {
        cv::Point3f {-4, 4, 0},
        cv::Point3f { 4, 4, 0},
        cv::Point3f { 4,-4, 0},
        cv::Point3f {-4,-4, 0}
    };

    struct RobotRelativeTagInfo {
        std::vector<cv::Mat1d> rvecs; 
        std::vector<cv::Mat1d> tvecs; 
        std::vector<int> ids;
    };
    RobotRelativeTagInfo RobotRelativePoseFromEstimationResult(const Apriltag::EstimationResult& esti) {
        const int count = esti.ids.size(); 
        std::vector<cv::Mat1d> rvecs; rvecs.reserve(count);
        std::vector<cv::Mat1d> tvecs; tvecs.reserve(count);
        std::vector<int> ids; ids.reserve(count);

        // solvepnp to generate cords
        for (int i = 0; i < count; i++) {
            int id = esti.ids[i];

            std::vector<cv::Mat1d> allRvec;
            std::vector<cv::Mat1d> allTvec; 
            cv::solvePnPGeneric(
                m_objectPoints,
                esti.corners[i], 
                esti.cameraInfo->matrix,
                esti.cameraInfo->distCoeffs,
                allRvec,
                allTvec,
                false, 
                cv::SOLVEPNP_SQPNP
            );
            // cv::solvePnP(
            //     m_objectPoints, 
            //     esti.corners[i], 
            //     esti.cameraInfo->matrix, 
            //     esti.cameraInfo->distCoeffs,
            //     rvec, 
            //     tvec, 
            //     false, 
            //     cv::SOLVEPNP_SQPNP); 
            // if (!ret) continue; // solve pnp "should" always return somehting

            // math
            int _c = allTvec.size();
            for (int j = 0; j < _c; j++) {
                cv::Mat1d rmat; 
                cv::Rodrigues(allRvec[j], rmat); 
                cv::Mat1d theta = h::rad2deg(h::rodRotMatToEuler(rmat));

                cv::Mat1d rmatT; 
                cv::transpose(rmat,rmatT);
                cv::Mat1d pmat = (rmatT * allTvec[j]) * -1; // element wise mutiplication of negated rmat row 0
                pmat *= APRILTAG_BLOCK_SIZE_cm;
                
                // pamt and theta is already flat for index access

                rvecs.push_back(std::move(theta));
                tvecs.push_back(std::move(pmat));
                ids.push_back(id); 
            }
            
            // auto& c = corners[i];
            // std::vector<double> relavtiveCorners {
            //     c[0].x / frameWidth, c[0].y / frameHeight,
            //     c[1].x / frameWidth, c[1].y / frameHeight,
            //     c[2].x / frameWidth, c[2].y / frameHeight,
            //     c[3].x / frameWidth, c[3].y / frameHeight,
            // };

        }

        return {.rvecs = rvecs, .tvecs = tvecs, .ids = ids};
    }


    std::pair<double, double> CamRelativeToAbsoulote(double cx, double cy, double tx, double ty, double tYaw) {
        double theta = h::NormalizeAngle(tYaw); 

        auto [x, y] = h::rotatePoint(cx, cy, theta); 

        return {tx + x, ty + y}; 
    } // CamRelativeToAbsoulote

    struct PoseArray {
        std::vector<double> xs; 
        std::vector<double> ys; 
        std::vector<double> rs; 
    };
    PoseArray RobotPoseFromEstimationResult(RobotRelativeTagInfo rr, std::shared_ptr<Camera::CameraData> camera) {
        const int count = rr.ids.size();
        PoseArray res; 
        res.xs.reserve(count); 
        res.ys.reserve(count); 
        res.rs.reserve(count); 

        auto& cameraPos = camera->cameraPos;

        for (int i = 0; i < count; i++) {
            Field::TagLocation tag = Field::tags[rr.ids[i]];
            
            auto& rvec = rr.rvecs[i];
            // cv::transpose(rr.rvecs[i], rvec);
            auto& tvec = rr.tvecs[i];

            // calculate yaw
            double yaw; 
            
            yaw = *rvec[1] - cameraPos[3];
            yaw = tag.yaw + yaw; 
            yaw = 180.0 + yaw; // adjust for opencv vs wpilib roatation cordinate systems
            yaw = h::NormalizeAngle(yaw);

            double camX = *tvec[2] - cameraPos[0]; // camera parallel/z is same as world x for horzontally mounted camera
            double camY = -*tvec[0] - cameraPos[1]; // camera through/x is same as world y

            // compute robot relative poses
            auto robotCords = CamRelativeToAbsoulote(camX, camY, tag.x, tag.y, tag.yaw); 

            fmt::println("{}, {}, {}", robotCords.first, robotCords.second, yaw);

            res.xs.push_back(robotCords.first); 
            res.ys.push_back(robotCords.second);
            res.rs.push_back(yaw);

        }
        return res;
    }


    inline double rjOA(const std::vector<double>& data) {
        const int count = data.size();
        if (count == 0) {
            return 0;
        } else if (count == 1) {
            return data[0];
        } else if (count == 2) {
            return h::average(data);
        }

        // sort low to high
        std::vector<double> sorted(data);
        std::sort(sorted.begin(), sorted.end());
        
        int quartileSize = count/4;
        int midIdx = count/2;
        double q1 = sorted[midIdx - quartileSize];
        double q3 = sorted[midIdx + quartileSize];
        double iqr = q3 - q1;

        double lowerBound = q1 - 1.5 * iqr; 
        double upperBound = q3 + 1.5 * iqr; 

        auto lowerPos = std::lower_bound(sorted.begin(), sorted.end(), lowerBound); 
        auto uppwerPos = std::upper_bound(sorted.begin(), sorted.end(), upperBound); 

        std::vector<double> normalized (lowerPos, uppwerPos); 

        return h::average(normalized); 
    }

    struct NormalizedPose {
        Pos2D robotFieldPose;
        std::vector<Pos2DwTag> tagPoses;  
    };
    NormalizedPose RejectOutliersAndAverage(const PoseArray& robotPoses, const RobotRelativeTagInfo& tagPoses) {
        NormalizedPose result; 
        
        const int count = tagPoses.ids.size();
        std::map<int, PoseArray> condensedTagPoses {}; 
        for (int i = 0; i < count; i++) {
            int id = tagPoses.ids[i];
            auto [itr, _] = condensedTagPoses.insert({id, {}}); // get or insert
            auto& poseArr = itr->second;

            poseArr.rs.push_back(tagPoses.rvecs[i](1));
            poseArr.xs.push_back(tagPoses.tvecs[i](2));
            poseArr.ys.push_back(tagPoses.tvecs[i](0)); 
        }

        for (auto& [k, v] : condensedTagPoses) {
            result.tagPoses.emplace_back(Pos2DwTag {
                .x = rjOA(v.xs),
                .y = rjOA(v.ys),
                .rot = rjOA(v.rs),
                .id = k,
            });
        }

        result.robotFieldPose.rot = rjOA(robotPoses.rs);
        result.robotFieldPose.x = rjOA(robotPoses.xs);
        result.robotFieldPose.y = rjOA(robotPoses.ys); 

        return result;        
    }

    // std::vector<Group> GroupCords(const std::vector<Apriltag::EstimationResult>& cords, double limitRadiCM = 50) {    
    //     std::vector<Group> groups; 

    //     // groups.emplace_back(cords[0], 1); // initialize with first cordinate 
    //     int totalNumOfCords = cords.size(); 
    //     for (int i = 0; i < totalNumOfCords; i++) {
    //         int currentNumOfGroups = groups.size();
    //         Pos2DwTag current = RobotPoseFromEstimationResult(cords[i]); 

    //         // calculate distance from current cord to all know groups 
    //         std::vector<double> distances;
    //         distances.reserve(currentNumOfGroups);  
    //         for (Group g : groups) {
    //             // find distance of tag to each group with Pathogram Therom
    //             distances.emplace_back(std::sqrt(std::pow(g.cord.x - current.x, 2) + std::pow(g.cord.y - current.y, 2))); 
    //         }

    //         // assign current cord to a group or make a new group based on distance
    //         bool isNewGroup = true; 
    //         // iterate through all distances to each group, check if distance < limitRadiCM
    //         for (int i = 0; i < currentNumOfGroups; i++) {
    //             double d = distances[i]; 
    //             if (d < limitRadiCM) { // the point is close enough to the group
    //                 // join the group if matched 
    //                 Group& g = groups[i]; 
    //                 g.cord.x = (g.cord.x + current.x)/2; // average the old cords with the new ones
    //                 g.cord.y = (g.cord.y + current.y)/2;
    //                 g.cord.rot = (g.cord.rot + current.rot)/2; // average yaw
    //                 g.count += 1; // increment count
    //                 g.tags.push_back(std::move(current)); // add the current tag to the list of good tags

    //                 isNewGroup = false; //unmark makeNewGroup
    //                 break;
    //             }
    //         }

    //         if (isNewGroup) { // make new group if needed
    //             groups.emplace_back(Group {
    //                 .cord = Pos2D(current.x, current.y, current.rot), 
    //                 .tags = std::vector<Pos2DwTag>{std::move(current)}, 
    //                 .count = 1}
    //             );
    //             continue; 
    //         }
    //     }
    //     return std::move(groups); 
    // }

    // // Finds the Group with the best score, returns the index to that group, the group can be accessed with groups[index]
    // int FindBestGroup(const std::vector<Group>& groups) {
    //     int bestResIndex = 0; 
    //     int lastBestScore = 0;
    //     int numOfGroups = groups.size();  
    //     for (int i = 0; i < numOfGroups; i++) {
    //         int numTagsInGroup = groups[i].count; 
    //         if (numTagsInGroup > lastBestScore) {
    //             bestResIndex = i; 
    //             lastBestScore = numTagsInGroup; 
    //         } // otherwise skip
    //     }
    //     return bestResIndex; 
    // }

    // double FindBestYaw(const std::vector<Pos2D>& cords) {
    //     std::vector<double> yaws;
    //     yaws.reserve(cords.size());
    //     for (const auto& cord : cords) {
    //         yaws.emplace_back(cord.rot); 
    //     }
    //     return h::average(h::reject_outliers_2(yaws));
    // }
} // Solvers