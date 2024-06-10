
#include "global.cpp"
#include "common.cpp"
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/opencv.hpp>

constexpr double APRILTAG_BLOCK_SIZE_cm = 2.0;  

namespace cobalt = boost::cobalt;

// implenmentaion of Rodugriz Rotation matrix to Euler angles based on
// https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
// or /docs/euler-angles
// (x,y,z)
cv::Mat1d rodRotMatToEuler(cv::Mat1d m) {
    double x = atan2(m[1][2], m[2][2]); 
    double c_2 = sqrt(pow(m[0][0],2) + pow(m[0][1],2));
    double y = atan2(-m[0][2], c_2); 
    double s_1 = sin(x);
    double c_1 = cos(x); 
    double z = atan2(s_1*m[2][0] - c_1 * m[1][0], c_1*m[1][1] - s_1*m[2][1]); 
    return cv::Mat1d{3} << x, y, z; 
}

constexpr double RAD2DEG_RATIO = 180.0/CV_PI;
cv::Mat rad2deg(cv::Mat src) {
    return src * RAD2DEG_RATIO; 
}

struct camera_MAIN_45FOV {
  int id = 0; 
  bool apriltags = true;
  // int camToRobotPos{4} = {800,250,300, 0}, # anchored at bottom right of robot
  int camToRobotPos[4] = {0,0,0, 0};
  // std::vector<std::vector<double>> matrix =
  //   {{710.8459662, 0, 584.09769116},
  //   {0.,710.64515618, 485.94212983},
  //   {0., 0., 1., }},
  // std::vector<std::vector<double>> distCoeffs =
  // {{-0.3689181,0.12470983,-0.0062236,0.00298559,-0.01839474}};
  cv::Matx<double, 3, 3> matrix = {673.49634849, 0, 616.93113106, 0, 670.71012973, 536.45109056, 0, 0, 1};
    // {{673.49634849, 0, 616.93113106},
    // {0, 670.71012973, 536.45109056},
    // {0, 0, 1}};
  cv::Matx<double, 1, 5> distCoeffs = 
    {-0.18422303, 0.04338743, -0.0010019, 0.00080675, -0.00543398};
    // {{-0.18422303, 0.04338743, -0.0010019, 0.00080675, -0.00543398}};
};

namespace Apriltag {
struct Estimation {
    int id; 
    cv::Mat1d rot; 
    cv::Mat1d trans; 
};
typedef std::vector<Estimation> AllEstimations; 

class Estimator {
    public:
        Estimator(): 
            detector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11)) 
        {
            
        }; // Estimator

        cobalt::promise<AllEstimations> Detect(cv::Mat image) {
            AllEstimations estimations; 

            // detecting the tags in image
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids; 
            this->detector.detectMarkers(image, corners, ids); 
            
            #ifdef DEBUG
            cv::aruco::drawDetectedMarkers(image, corners, ids); 
            cv::imshow("markers", image); 
            #endif
            // solvepnp to generate cords
            int maxI = ids.size();
            for (int i = 0; i < maxI; i++) {
                cv::Mat1d rvec;
                cv::Mat1d tvec; 
                // cv::Mat objPoints(4, 1, CV_32FC3);
                // objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-4.f, 4.f, 0);
                // objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(4.f, 4.f, 0);
                // objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(4.f, -4.f, 0);
                // objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-4.f, -4.f, 0);

                bool ret = cv::solvePnP(
                    objectPoints, 
                    corners[i], 
                    cameraData.matrix, 
                    cameraData.distCoeffs,
                    rvec, 
                    tvec); 
                if (!ret) continue;

                // math
                cv::Mat1d rmat; 
                cv::Rodrigues(rvec, rmat); 
                cv::Mat1d theta = rad2deg(rmat.row(0));

                cv::Mat1d rmat0T; 
                cv::transpose(rmat.row(0),rmat0T);
                cv::Mat1d pmat = (-rmat0T).mul(tvec); // element wise mutiplication of negated rmat row 0
                pmat.reshape(1); //flatten
                pmat *= APRILTAG_BLOCK_SIZE_cm;
                
                 
                estimations.push_back(Estimation {
                    .id = ids[i],
                    .rot = theta, 
                    .trans = pmat
                }); 
            }
            co_return std::move(estimations); 
        }; // Detect


    private:
        camera_MAIN_45FOV cameraData; 
        cv::aruco::ArucoDetector detector; 
        std::vector<cv::Point3f> objectPoints = {
            {-4, 4, 0}, // every one is 33 mm
            { 4, 4, 0},
            { 4,-4, 0},
            {-4,-4, 0}
        };

};
}// namespace Apriltag