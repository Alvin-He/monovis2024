#pragma once
// absoulote position tracking
#include "Solvers.cpp"
#include "Field.cpp"
#include "TypeDefs.cpp"
#include <cstddef>
#include <map>
#include <optional>
#include <vector>
namespace World {
    // Tracking for everything known in this world
    class RobotPose {
        public:
        RobotPose() = default;

        void Update(const std::vector<Apriltag::EstimationResult>& estimationResults) {
            if (estimationResults.size()<1) return; 

            // find most accurate robot cord
            std::vector<Group> cordGroups = Solvers::GroupCords(estimationResults); 
            int bestPoseIdx = Solvers::FindBestGroup(cordGroups); 
            Group& bestGroup = cordGroups[bestPoseIdx];
            // bestPose.rot = FindBestYaw(poses); // doesn't really make sense to find best yaw seperately if we are throwing away most other non useful cords 

            m_lastRobotPose = bestGroup.cord;
        } // Update

        Pos2D GetRobotPose() {
            return m_lastRobotPose; 
        }
        
        std::optional<Pos2DwTag> GetTransformationToTag(int id) {
            if (! Field::tags.contains(id)) return std::nullopt;
            
            auto& tagLoc = Field::tags[id];
            
            return Pos2DwTag {
                .x = tagLoc.x - m_lastRobotPose.x,
                .y = tagLoc.y - m_lastRobotPose.y,
                .rot = tagLoc.yaw - m_lastRobotPose.rot,
                .id = id
            };
        }

        std::vector<Pos2DwTag> GetTransformationToAllTags() {
            std::vector<Pos2DwTag> resVec; 
            for (auto& tag : Field::tags) {
                resVec.emplace_back(
                    tag.second.x - m_lastRobotPose.x,
                    tag.second.y - m_lastRobotPose.y, 
                    tag.second.yaw - m_lastRobotPose.rot,
                    tag.first
                );
            }
            return resVec;
        }

        private:
            Pos2D m_lastRobotPose = {.x = 0, .y=0, .rot = 0}; 

    }; // class World

} 
