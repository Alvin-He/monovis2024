#pragma once
#include "common.cpp"
#include "global.cpp"
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/cudawarping.hpp>
#include <boost/cobalt.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "network/network_time.cpp"
namespace cobalt = boost::cobalt;

namespace Camera
{

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

        void ReadLoop(std::vector<cv::Mat>& frameBufs, std::vector<int64_t>& tsBufs, std::atomic<int>& usuable) {
            boost::thread thr ([&] {
                while(!f_exit) {
                    auto frame = Read(); 
                    auto ts = NetworkTime::Now();
                    frame = Camera::Resize(frame, K::PROC_FRAME_SIZE);
                    if (usuable == 1) {
                        frameBufs[0] = std::move(frame); 
                        tsBufs[0] = std::move(ts);
                        usuable = 0;
                    } else {
                        frameBufs[1] = std::move(frame);
                        tsBufs[1] = std::move(ts); 
                        usuable = 1;
                    }
                }
            });
            thr.detach(); 
        }

        private:
            cv::VideoCapture m_cap; 
    }; // Camera::FrameGenerator
} // namespace Camera
 