

#include <chrono>
#include "common.cpp"
#include "global.cpp"
#include "const.cpp"
#include "camera/Camera.cpp"
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <vector>
#include <fmt/include/fmt/chrono.h>
#include <fmt/include/fmt/ranges.h>
#include "network/ReadNetworkFrame.cpp"

#undef HAS_CUDA

#ifdef HAS_CUDA
#include <opencv4/opencv2/cudaarithm.hpp>
#include <opencv4/opencv2/cudaimgproc.hpp>
#include <opencv4/opencv2/cudafilters.hpp>
#endif

cobalt::main co_main(int argc, char* argv[]) {
    double kRectToFrameMinimalRatio = 0.001;


    // Tracking point camera, directions are computed based on this
    cv::Point trackPoint {320,240};

    // Define the range for purple color in HSV
    cv::Scalar lowerPurple(130, 50, 50); // Adjust the values as needed
    cv::Scalar upperPurple(160, 255, 255);

    const cv::Mat morphologyKernel3x3 = cv::getStructuringElement(cv::MORPH_ERODE, {3,3});
    #ifdef HAS_CUDA
    cv::Ptr<cv::cuda::Filter> cudaErodeFilter = cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, morphologyKernel3x3);
    cv::Ptr<cv::cuda::Filter> cudaDilateFilter = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, morphologyKernel3x3);
    
    cv::cuda::GpuMat gpuFrame, gpuMask; // precreated GpuMat objects
    #endif

    while (true) {
    fmt::println("\n");

    // read frames from network    
    cv::Mat frame;
    if (auto ret = co_await Network::ReadFrame("127.0.0.1", "8081")) {
        frame = ret.value(); 
    } else {
        continue;
    }

    auto start = std::chrono::high_resolution_clock::now(); // performance timer

    cv::Mat mask; 

    #ifndef HAS_CUDA
    // resize to process size
    cv::resize(frame, frame, K::PROC_FRAME_SIZE);

    // filtering 
    cv::Mat hsvFrame;
    cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

    // threshholding 
    cv::inRange(hsvFrame, lowerPurple, upperPurple, mask);
    
    // erode & dilate to reduce noise 
    cv::erode(mask, mask, morphologyKernel3x3);
    cv::dilate(mask, mask, morphologyKernel3x3);
    
    #else // cuda equiviliant of cpu code above, for performance reasons 
    gpuFrame.upload(frame);

    cv::cuda::resize(gpuFrame, gpuFrame, K::PROC_FRAME_SIZE); // not using Camera::Resize cause this avoids one download from gpu
    gpuFrame.download(frame);

    cv::cuda::cvtColor(gpuFrame, gpuFrame, cv::COLOR_BGR2HSV);

    cv::cuda::inRange(gpuFrame, lowerPurple, upperPurple, gpuMask);

    cudaErodeFilter->apply(gpuMask, gpuMask); // apprently filters can be slower than cpu, so please check performance numbers
    cudaDilateFilter->apply(gpuMask, gpuMask);

    gpuMask.download(mask);
    // cv::erode(mask, mask, morphologyKernel3x3); // experiment with doing morph on jetson nano gpu, could be slower or faster
    // cv::dilate(mask, mask, morphologyKernel3x3);
    #endif // END CUDA CODE

    cv::imshow("mask", mask);
    
    fmt::println("To Mask Finish: {:%S}", std::chrono::high_resolution_clock::now() - start); 
    
    // find all contours in frame
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); 

    fmt::println("To Find Contors Finish: {:%S}", std::chrono::high_resolution_clock::now() - start); 

    // cv::drawContours(frame, contours, -1, {3, 215, 252});

    // generate bounding rectangles around those contours
    std::vector<cv::Rect> rects; 
    for (auto& contour : contours) {
        auto rect = cv::boundingRect(contour);
        rects.push_back(rect);
    }

    // group contour BBoxes together based on if they occopuy the same horizontal space as another contour
    // we do this because the reef is only verticle, therefore if we group based on horizontal space, 
    // we can generate a bbox for an entire branch of the reef without having to deal with algae or coral
    // this also groups coral that's next to each other together as their contours often occopuy the same horizontal space when 
    // looked at from a camera 
    std::vector<cv::Rect> groupRects; 
    for (auto& rect : rects) {

        bool newGroup = true;
        for (auto& g : groupRects) {

            if ((g.x <= rect.x + rect.width) && // ends after start
                (rect.x <= g.x + g.width) // starts before end     
            ) {
                
                int xEnd = g.x + g.width;
                if (rect.x < g.x) 
                    g.x = rect.x;
                if (rect.x + rect.width > xEnd)
                    xEnd = rect.x + rect.width;  
                g.width = xEnd - g.x;

                int yEnd = g.y + g.height;
                if (rect.y < g.y) 
                    g.y = rect.y; 
                if (rect.y + rect.height > yEnd)
                    yEnd = rect.y + rect.height;
                g.height = yEnd - g.y;

                newGroup = false;
                break;
            }

        }
        
        if (newGroup) groupRects.push_back(rect);
    }
    
    // targeting cursor verticle line to mark where the trackPoint is
    cv::line(frame, {trackPoint.x, trackPoint.y-10}, {trackPoint.x, trackPoint.y+10}, {0, 179, 255});

    // go thorough all the groups and find the one we are tracking and generate instructions on what the driver should do
    for (int i = 0; i < groupRects.size() ; i++) {//&& i <= 5
        auto& rect = groupRects[i];

        // basic filtering based on contour size
        if ((double)rect.area()/(frame.size().area()) < kRectToFrameMinimalRatio) {
            continue; // skip if contour is less than 0.1% camera view
        }

        // draw the bbox around the grouped reef objects
        cv::rectangle(frame, groupRects[i], {3, 215, 252}, 1); 
        cv::putText(frame, "Reef", {rect.x, rect.y + rect.height}, 1, 1, {0, 255,0});

        // if trackpoint is with in the horizontal space that this group occopuies
        if (rect.x < trackPoint.x && trackPoint.x < rect.x + rect.width) {
            cv::Mat roi = mask(rect); // region of interest that this bbox describes 

            // get moment of all contours described by the bbox area
            // google image moment iywtk, it's a bunch of numbers describing a black shape.
            cv::Moments moment = cv::moments(roi, true);
            double cx = moment.m10/moment.m00;
            double cy = moment.m01/moment.m00;
    
            cv::Point2d momentCenter {rect.x + cx, rect.y + cy};
            cv::circle(frame, momentCenter, 3, {255, 179, 0}, -1);
    
            // determine what to do based on distance of the bbox center to moment center
            // because the majority of reef is verticle, the moment naturally goes toward the verticle part of reef
            // The bbox center is going to be away from the moment center if we are not aligned straight to the reef
            // therefore by looking at the bbox center difference to the moment center we can find out if we need to move left or right
            // based on if the bbox center is to the left or right of the moment center 
            cv::Point2d rectCenter(rect.x + rect.width/2.0, rect.y + rect.height/2.0);
            if (std::abs(momentCenter.x - rectCenter.x) < 1) {
                fmt::println("done");
                cv::line(frame, {trackPoint.x, trackPoint.y-10}, {trackPoint.x, trackPoint.y+10}, {0, 255,0});
            } else if (momentCenter.x > rectCenter.x) {
                cv::circle(frame, rectCenter, 3, {255, 0, 0}, -1);
                cv::line(frame, {trackPoint.x-5, trackPoint.y}, {trackPoint.x, trackPoint.y}, {0, 0, 255}, 2);
                fmt::println("left");
            } else {
                cv::circle(frame, rectCenter, 3, {0, 0, 255}, -1);
                cv::line(frame, {trackPoint.x, trackPoint.y}, {trackPoint.x+5, trackPoint.y}, {255, 0, 0}, 2);
                fmt::println("right");
            }


            cv::rectangle(frame, rect, {0, 255, 0}, 1); 
        }
        // fmt::println("x {}, y: {}, w: {}, h: {}", groupRects[i].x, groupRects[i].y, groupRects[i].width, groupRects[i].height);
    }


    // cv::rectangle(frame, r, {0, 0, 200}, 2); 

    cv::imshow("test", frame);

    fmt::println("Took: {:%S}", std::chrono::high_resolution_clock::now() - start); 

    // press ESC to exit
    int code = cv::waitKey(20) & 0xff;
    if (code == 27) break;
    }
    co_return 0;
}