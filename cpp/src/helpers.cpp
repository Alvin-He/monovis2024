#pragma once
#include "fmt/include/fmt/std.h"
#include <array>
#include <cmath>
#include <numeric>
#include <opencv4/opencv2/opencv.hpp>
#include <boost/cobalt.hpp>
#include <algorithm>
#include <Eigen/Dense>
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
cv::Mat rodRotMatToEuler(const cv::Mat1d& m) {
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
            result.emplace_back(data(i)); 
        }
    }
    return result; 
    // return result; 
    // d = np.abs(data - np.median(data))
    
    // mdev = np.median(d)
    // s = d / (mdev if mdev else 1.)
    // return data[s < m]
}

template<typename T>
T average(const std::vector<T>& data) {
    if (data.empty()) return 0;
    T sum = std::reduce(data.begin(), data.end()); 
    T size = static_cast<T>(data.size());
    return sum/size; 
}

double average(const vector_d& data) {
    return average<double>(data);
}

std::array<double, 2> rotatePoint(double x, double y, double theta) {
    // origional point -> polar cords -> add theta to the polar cords -> convert back to normal cords for res
    theta = theta * DEG2RAD_RATIO;
    double h = std::sqrt(x*x + y*y);
    double a = std::atan(y/x);
    double p = a + theta;
    if (x < 0) p -= CV_PI;
    double fX = h * std::cos(p);
    double fY = h * std::sin(p);
    return {fX, fY};
}  

cv::Mat rad2deg(cv::Mat src) {
    return src * RAD2DEG_RATIO; 
}

template<typename T>
T rad2deg(T src) {
    return src * RAD2DEG_RATIO;
}

double NormalizeAngle(double s) {
    return std::remainder(s, 360); 
}

};