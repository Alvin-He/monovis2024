#pragma once
#include <opencv4/opencv2/opencv.hpp>
#include <boost/cobalt.hpp>

// alialias
namespace cobalt = boost::cobalt;

constexpr double RAD2DEG_RATIO = 180.0/CV_PI;
constexpr double DEG2RAD_RATIO = CV_PI/180.0; 

// helper functions

// implenmentaion of Rodugriz Rotation matrix to Euler angles based on
// https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
// or /docs/euler-angles
// (x,y,z)
cv::Mat rodRotMatToEuler(cv::Mat1d m) {
    double x = atan2(m[1][2], m[2][2]); 
    double c_2 = sqrt(pow(m[0][0],2) + pow(m[0][1],2));
    double y = atan2(-m[0][2], c_2); 
    double s_1 = sin(x);
    double c_1 = cos(x); 
    double z = atan2(s_1*m[2][0] - c_1 * m[1][0], c_1*m[1][1] - s_1*m[2][1]); 
    return cv::Mat(cv::Matx<double, 1, 3> {x, y, z}); 
}

std::vector<double> rotatePoint(double x, double y, double theta) {
    theta = theta * DEG2RAD_RATIO;
    double r1 = sqrt(pow(x, 2) + pow(y, 2)); 
    double xp1 = r1*cos(atan(y/x) + theta); 
    double yp1 = r1*sin(atan(y/x) + theta); 
    return std::vector<double> {xp1, yp1}; 
}  

cv::Mat rad2deg(cv::Mat src) {
    return src * RAD2DEG_RATIO; 
}


double NormalizeAngle(double s) {
    return remainder(s, 360); 
}
