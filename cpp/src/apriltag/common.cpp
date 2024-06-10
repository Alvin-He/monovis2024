#include <boost/cobalt.hpp>

struct Tag {
    int id;
};
struct RobotPoseToTag {
    int id; 
    double x; 
    double y; 
    double z; 
    double rot;
}; 