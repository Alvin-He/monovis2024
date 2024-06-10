
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/cudawarping.hpp>
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>


namespace cobalt = boost::cobalt;

namespace Camera
{
    cobalt::generator<cv::Mat> Reader(cv::VideoCapture cap) {
        while (cap.isOpened())
        {
            cv::Mat res; 
            cap.read(res); 
            co_yield res; 
        }
    }// Camera::Reader

    cobalt::promise<cv::Mat> CudaResize(cv::Mat src, cv::Size2d size) {
        cv::cuda::GpuMat buf; 
        buf.upload(src); 
        cv::cuda::resize(buf, buf, size); 
        buf.download(src); 
        co_return src; 
    }// Camera::CudaResize
    
} // namespace Camera
 