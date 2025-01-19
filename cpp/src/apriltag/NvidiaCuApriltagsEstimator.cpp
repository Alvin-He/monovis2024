#pragma once

/////////////
// DEFUNCT //
/////////////

#ifdef HAS_CUDA

// IMPORTANT //
#error "NvidiaCuApriltagsEstimator is still in an old monovis architecture and currently have a broken implenmentation. Please DO NOT USE and use OpenCVArucoEstimator instead!"
//////////////


#include "common.cpp"
#include "Estimator.ipp"

#include <opencv2/core/cuda.hpp>
#include "helpers.cpp"
#include "cuAprilTags.h"
#include <cuda_runtime.h>

// and it was fine yesterday
namespace Apriltag {

class CUDAEstimator : Estimator {
    public:
        CUDAEstimator(std::shared_ptr<Camera::CameraData> cameraData, cv::aruco::DetectorParameters detectorParams): 
            m_cameraData(cameraData), 
            m_cuCamIntrinsics({
                .fx = (float_t)m_cameraData->matrix(0,0),
                .fy = (float_t)m_cameraData->matrix(1,1),
                .cx = (float_t)m_cameraData->matrix(0,2),
                .cy = (float_t)m_cameraData->matrix(1,2),
            })
        {
            // cudaStreamCreate(&m_mainStream);
            // size_t size = m_cameraData->calibratedAspectRatio.area() * 3 * sizeof(char); 
            // cudaMallocManaged(&m_cudaFrameBufPtr, size); 
            int ret = nvCreateAprilTagsDetector(&m_cuHandle, m_cameraData->calibratedAspectRatio.width, m_cameraData->calibratedAspectRatio.height, 5, cuAprilTagsFamily::NVAT_TAG36H11, &m_cuCamIntrinsics, 8);
            if (!(ret == 0)) { 
                fmt::println("Failed to create detector");
                throw 1; 
            } 
        }; // Estimator

        std::vector<EstimationResult> DetectCUDA(cv::Mat image) {
            std::vector<EstimationResult> estimations; 

            // cudaStreamAttachMemAsync(m_mainStream, image.data, image.size().area()*3*sizeof(char));
            // cudaStreamSynchronize(m_mainStream);
            // cudaMemcpy(m_cudaFrameBufPtr, image.data, image.size().area()*3*sizeof(char), cudaMemcpyKind::cudaMemcpyHostToDevice); 
            m_cudaImgBuf.upload(image);

            cuAprilTagsImageInput_t cuCamInput {
                .dev_ptr = reinterpret_cast<uchar3*>(m_cudaImgBuf.cudaPtr()), 
                .pitch = static_cast<size_t>(m_cudaImgBuf.step), 
                .width = static_cast<uint16_t>(m_cudaImgBuf.cols), 
                .height = static_cast<uint16_t>(m_cudaImgBuf.rows),
            };

            uint32_t num_tags = 0;
            cuAprilTagsDetect(m_cuHandle, &cuCamInput, m_cuResultBuf, &num_tags, k_maxNumCuResults, 0);

            // imagebuf.release();
            
            // detecting the tags in image            
            #if defined(DEBUG) && defined(GUI)
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids; 
            for (int i = 0; i < num_tags; i++) {
                auto&& tag = m_cuResultBuf[i];
                std::vector<cv::Point2f> tagCorners; 
                for (auto&& corner : tag.corners) {
                    tagCorners.emplace_back(corner.x, corner.y);                     
                }
                corners.push_back(tagCorners);
                ids.push_back(tag.id);
            }

            cv::aruco::drawDetectedMarkers(image, corners, ids); 
            cv::imshow(fmt::format("camera-{} markers", m_cameraData->id), image); 
            #endif

            // solvepnp to generate cords
            estimations.reserve(num_tags);
            for (int i = 0; i < num_tags; i++) {
                // cv::Mat1d rvec;
                cv::Mat tvec = cv::Mat(3, 1, CV_32F, &m_cuResultBuf[i].translation);// load cuApriltag translation data
                // cv::solvePnP(
                //     m_objectPoints, 
                //     corners[i], 
                //     m_cameraData->matrix, 
                //     m_cameraData->distCoeffs,
                //     rvec, 
                //     tvec, 
                //     false, 
                //     cv::SOLVEPNP_SQPNP); 
                // // if (!ret) continue; // solve pnp "should" always return somehting

                // math
                cv::Mat rmat = cv::Mat(3, 3, CV_32F, &m_cuResultBuf[i].orientation).t(); // load cuApriltag orientation data and transpose it to be row major for opencv

                // cv::Rodrigues(rvec, rmat); 
                cv::Mat theta = h::rad2deg(h::rodRotMatToEuler(rmat));

                cv::Mat rmatT; 
                cv::transpose(rmat,rmatT);
                cv::Mat pmat = (rmatT * tvec) * -1; // element wise mutiplication of negated rmat row 0
                pmat *= APRILTAG_BLOCK_SIZE_cm;
                
                // pamt and theta is already flat for index access

                estimations.emplace_back(m_cameraData, m_cuResultBuf[i].id, theta, pmat); 
            }
            estimations.shrink_to_fit();
            return std::move(estimations); 
        }

    private:
        std::shared_ptr<Camera::CameraData> m_cameraData; 
        cuAprilTagsHandle m_cuHandle;
        cuAprilTagsCameraIntrinsics_t m_cuCamIntrinsics; 
        cudaStream_t m_mainStream;
        cv::cuda::GpuMat m_cudaImgBuf; 
        const int k_maxNumCuResults = 100;
        cuAprilTagsID_t m_cuResultBuf[100]; 

};
}// namespace Apriltag

#endif