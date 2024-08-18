#pragma once
#include "common.cpp"
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/cudawarping.hpp>
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>


namespace cobalt = boost::cobalt;

namespace Camera
{
    class FrameGenerator {
        public:
        FrameGenerator (cv::VideoCapture cap) : m_cap(std::move(cap)) {};
        cv::Mat Read() {
            cv::Mat res;
            m_cap.read(res); // returns empty if unable to read
            return res;
        };
        cobalt::promise<cv::Mat> PromiseRead() {co_return std::move(Read());} 
        cobalt::task<cv::Mat> TaskRead() {co_return std::move(Read());}

        private:
            cv::VideoCapture m_cap; 
    }; // Camera::FrameGenerator

    cv::Mat Resize(cv::Mat src, cv::Size2d size) {
        #ifdef HAS_CUDA
            cv::cuda::GpuMat buf; 
            buf.upload(src); 
            cv::cuda::resize(buf, buf, size); 
            buf.download(src); 
            return src; 
        #else
            cv::resize(src, src, size);
            return src;
        #endif
    } // Camera::Resize
    cobalt::promise<cv::Mat> PromiseResize(cv::Mat& src, cv::Size2d& size) { co_return Resize(src, size); }
    cobalt::task<cv::Mat> TaskResize(cv::Mat& src, cv::Size2d& size) { co_return Resize(src, size); }
   
} // namespace Camera
 