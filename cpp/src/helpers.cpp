#pragma once
#include <opencv4/opencv2/opencv.hpp>
#include <boost/cobalt.hpp>
#include <algorithm>
#include <Eigen/Dense>
#include <cmath>
// alialias
namespace cobalt = boost::cobalt;

constexpr double RAD2DEG_RATIO = 180.0/CV_PI;
constexpr double DEG2RAD_RATIO = CV_PI/180.0; 
typedef std::vector<double> vector_d; 

namespace h {
// helper functions

//https://stackoverflow.com/a/62698308
template<typename Derived>
typename Derived::Scalar median( Eigen::DenseBase<Derived>& d ){
    auto r { d.reshaped() };
    std::sort( r.begin(), r.end() );
    return r.size() % 2 == 0 ?
        r.segment( (r.size()-2)/2, 2 ).mean() :
        r( r.size()/2 );
}
//https://stackoverflow.com/a/62698308
template<typename Derived>
typename Derived::Scalar median( const Eigen::DenseBase<Derived>& d ){
    typename Derived::PlainObject m { d.replicate(1,1) };
    return median(m);
}

// implenmentaion of Rodugriz Rotation matrix to Euler angles based on
// https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
// or /docs/euler-angles
// (x,y,z)
cv::Mat rodRotMatToEuler(cv::Mat1d m) {
    double x = std::atan2(m[1][2], m[2][2]); 
    double c_2 = std::sqrt(std::pow(m[0][0],2) + std::pow(m[0][1],2));
    double y = std::atan2(-m[0][2], c_2); 
    double s_1 = std::sin(x);
    double c_1 = std::cos(x); 
    double z = std::atan2(s_1*m[2][0] - c_1 * m[1][0], c_1*m[1][1] - s_1*m[2][1]); 
    return cv::Mat(cv::Matx<double, 3, 1> {x, y, z}); 
}

// https://stackoverflow.com/a/45399188 #1.2 works pretty well to kick out more than ~5 cm of difference
vector_d reject_outliers_2(const vector_d& linearArray, double m = 1.2) {
    Eigen::ArrayXd data = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(linearArray.data(), linearArray.size()).array(); 
    double med = median(data); 
    Eigen::ArrayXd d = (data.array() - med).abs(); 

    double mdev = median(d); 
    if (mdev != 0) {
        d /= mdev; 
    } 
    
    auto comparsionResult = d < m; 
    vector_d result; 
    for (int i = 0; i < comparsionResult.size(); i++) {
        if (comparsionResult(i)) {
            result.push_back(data(i)); 
        }
    }
    return result; 
    // return result; 
    // d = np.abs(data - np.median(data))
    
    // mdev = np.median(d)
    // s = d / (mdev if mdev else 1.)
    // return data[s < m]
}

double average(const vector_d& data) {
    double sum = std::accumulate(data.begin(), data.end(), 0.0); 
    double size = data.begin() - data.end();
    return sum / size; 
}

std::vector<double> rotatePoint(double x, double y, double theta) {
    theta = theta * DEG2RAD_RATIO;
    double r1 = std::sqrt(std::pow(x, 2) + std::pow(y, 2)); 
    double xp1 = r1*std::cos(std::atan(y/x) + theta); 
    double yp1 = r1*std::sin(std::atan(y/x) + theta); 
    return std::vector<double> {xp1, yp1}; 
}  

cv::Mat rad2deg(cv::Mat src) {
    return src * RAD2DEG_RATIO; 
}


double NormalizeAngle(double s) {
    return std::remainder(s, 360); 
}

};