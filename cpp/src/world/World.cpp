#pragma once
// absoulote position tracking
#include "Solvers.cpp"
#include "Field.cpp"
#include "TypeDefs.cpp"
#include <iterator>
#include <map>
#include <optional>
#include <vector>
namespace World {
    // Tracking for everything known in this world
    class RobotPose {
        public:
        RobotPose() = default;

        void Update(std::vector<Apriltag::EstimationResult> estimationResults) {
            if (estimationResults.size()<1) return; 

            Solvers::RobotRelativeTagInfo tagInfos;
            Solvers::PoseArray poseArr;  
            for (auto& res : estimationResults) {
                if (res.ids.size() <= 0) continue;
                auto currentTagPoses = Solvers::RobotRelativePoseFromEstimationResult(res);
                auto robotPoses = Solvers::RobotPoseFromEstimationResult(currentTagPoses, res.cameraInfo); 

                tagInfos.rvecs.insert(tagInfos.rvecs.end(), std::make_move_iterator(currentTagPoses.rvecs.begin()), std::make_move_iterator(currentTagPoses.rvecs.end()));
                tagInfos.tvecs.insert(tagInfos.tvecs.end(), std::make_move_iterator(currentTagPoses.tvecs.begin()), std::make_move_iterator(currentTagPoses.tvecs.end()));
                tagInfos.ids.insert(tagInfos.ids.end(), std::make_move_iterator(currentTagPoses.ids.begin()), std::make_move_iterator(currentTagPoses.ids.end()));
                poseArr.rs.insert(poseArr.rs.end(), std::make_move_iterator(robotPoses.rs.begin()), std::make_move_iterator(robotPoses.rs.end()));
                poseArr.xs.insert(poseArr.xs.end(), std::make_move_iterator(robotPoses.xs.begin()), std::make_move_iterator(robotPoses.xs.end()));
                poseArr.ys.insert(poseArr.ys.end(), std::make_move_iterator(robotPoses.ys.begin()), std::make_move_iterator(robotPoses.ys.end())); 
            }
            // estimationResults is now consumed and in an indeterminate state
            
            if (tagInfos.ids.size() <= 0) return; // early exit if no tags were found

            // find most accurate robot cord
            m_lastResult = Solvers::RejectOutliersAndAverage(poseArr, tagInfos);
            // std::vector<Group> cordGroups = Solvers::GroupCords(estimationResults); 
            // int bestPoseIdx = Solvers::FindBestGroup(cordGroups); 
            // Group& bestGroup = cordGroups[bestPoseIdx];
            // bestPose.rot = FindBestYaw(poses); // doesn't really make sense to find best yaw seperately if we are throwing away most other non useful cords 
        } // Update

        Solvers::NormalizedPose GetLastResult() {
            return m_lastResult; 
        }
        
        private:
            Solvers::NormalizedPose m_lastResult; 

    }; // class World

} 
