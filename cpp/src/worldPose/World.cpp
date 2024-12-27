#pragma once
// absoulote position tracking
#include "Solvers.cpp"
#include "TypeDefs.cpp"
#include <vector>
namespace WorldPose {
    // Tracking for everything known in this world
    class World {
        public:
        World() = default;

        void Update(const std::vector<Apriltag::EstimationResult>& estimationResults) {
            if (estimationResults.size()<1) return; 

            // find most accurate robot cord
            std::vector<Group> cordGroups = Solvers::GroupCords(estimationResults); 
            Pos2D bestPose = Solvers::FindBestCord(cordGroups); 
            // bestPose.rot = FindBestYaw(poses); // doesn't really make sense to find best yaw seperately if we are throwing away most other non useful cords 

            m_lastRobotPose = bestPose;
        } // Update

        Pos2D GetRobotPose() {
            return m_lastRobotPose; 
        }
        
        private:
            Pos2D m_lastRobotPose = {.x = 0, .y=0, .rot = 0}; 

    }; // class World

} 
