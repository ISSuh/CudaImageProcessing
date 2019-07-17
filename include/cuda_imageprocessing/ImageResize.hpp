#ifndef IMAGE_RESIZE_H
#define IMAGE_RESIZE_H

// interface
#include "ImageProcessing.hpp"

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// CUDA NPP
#include <npp.h>

// Error Check
#include <cuda_imageprocessing/ErrorCheck.hpp>

namespace ips{

class ImageResize : public ImageProcessing{
public: 
    ImageResize(const int dstW, const int dstH){
        m_dstW = dstW;
        m_dstH = dstH;

        m_dstSize.width = m_dstW;
        m_dstSize.height = m_dstH;

        m_dstROI = {0, 0, m_dstW, m_dstH};
    }

    bool Run(const sensor_msgs::Image &srcMsg, sensor_msgs::Image &dstMsg) {
        if (!InitialMember(srcMsg))
            return false;

        std::vector<uint8_t> resizedImage(m_dstStep * m_dstH, 0);

        {
            void *srcImage = nullptr, *dstImage = nullptr;

            if (!CudaAllocation(&srcImage, &dstImage))
                return false;

            if (CheckCUDA(cudaMemcpy2D(srcImage, m_srcStep, &srcMsg.data[0], m_srcStep,   m_srcW * m_srcChannelNum, m_srcH, cudaMemcpyHostToDevice)) != 0)
                return false;

            if (!Processing(&srcImage, &dstImage))
                return false;

            if (CheckCUDA(cudaMemcpy2D(&resizedImage[0], m_dstW * m_srcChannelNum, dstImage,   m_dstStep, m_dstW * m_srcChannelNum, m_dstH, cudaMemcpyDeviceToHost)) != 0)
                return false;

            nppiFree(srcImage);
            nppiFree(dstImage);
        }

        {
            dstMsg.header.frame_id = srcMsg.header.frame_id;
            dstMsg.width = m_dstW;
            dstMsg.height = m_dstH;
            dstMsg.step = m_dstStep;
            dstMsg.encoding = srcMsg.encoding;
            dstMsg.is_bigendian = srcMsg.is_bigendian;

            dstMsg.data = std::move(resizedImage);
        }

        return true;
    };

protected:
    virtual bool InitialMember(const sensor_msgs::Image &srcMsg) {
        if ((srcMsg.width <= 0) || (srcMsg.height <= 0)){
            ROS_ERROR("Unvalid image size. check your image.");
            return false;
        }

        if (srcMsg.encoding != "rgb8"   &&
            srcMsg.encoding != "rgb16"  &&
            srcMsg.encoding != "rgba8"  &&
            srcMsg.encoding != "rgba16" &&
            srcMsg.encoding != "bgr8"   &&
            srcMsg.encoding != "bgr16"  &&
            srcMsg.encoding != "bgra8"  &&
            srcMsg.encoding != "bgra16" &&
            srcMsg.encoding != "mono8"  &&
            srcMsg.encoding != "mono16"){

            ROS_ERROR("Invalid Encording value! Not supportted Encording Type.");
            return false;
        }

        m_srcW = srcMsg.width;
        m_srcH = srcMsg.height;
        m_srcSize.width = srcMsg.width;
        m_srcSize.height = srcMsg.height;
        m_srcROI = {0, 0, m_srcW, m_srcH};

        m_srcEncording = srcMsg.encoding;
        m_srcChannelNum = sensor_msgs::image_encodings::numChannels(m_srcEncording);

        m_srcStep = m_srcW * m_srcChannelNum;
        m_dstStep = m_dstW * m_srcChannelNum;

        return true;
    };

    virtual bool CudaAllocation(void **src, void **dst) {
        int srcStep, dstStep;

        if (m_srcEncording == "mono8"){
            *src = nppiMalloc_8u_C1(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_8u_C1(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else if (m_srcEncording == "mono16"){
            *src = nppiMalloc_16u_C1(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_16u_C1(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else if (m_srcEncording == "rgb8" || m_srcEncording == "bgr8"){
            *src = nppiMalloc_8u_C3(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_8u_C3(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else if (m_srcEncording == "rgb16" || m_srcEncording == "bgr16"){
            *src = nppiMalloc_16u_C3(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_16u_C3(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else if (m_srcEncording == "rgba8" || m_srcEncording == "bgra8"){
            *src = nppiMalloc_8u_C4(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_8u_C4(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else if (m_srcEncording == "rgba16" || m_srcEncording == "bgra16"){
            *src = nppiMalloc_16u_C4(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_16u_C4(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else
            return false;
    };

    virtual bool Processing(void **src, void **dst) {
         if (m_srcEncording == "mono8"){
        if (CheckNPP(nppiResize_8u_C1R((Npp8u *)*src, m_srcStep, m_srcSize, m_srcROI,
                                            (Npp8u *)*dst, m_dstStep, m_dstSize, m_dstROI,
                                            NPPI_INTER_LINEAR)) != 0)
            return false;

        return true;
        }
        else if (m_srcEncording == "mono16"){
            if (CheckNPP(nppiResize_16u_C1R((Npp16u *)*src, m_srcStep, m_srcSize, m_srcROI,
                                                 (Npp16u *)*dst, m_dstStep, m_dstSize, m_dstROI,
                                                 NPPI_INTER_LINEAR)) != 0)
                return false;

            return true;
        }
        else if (m_srcEncording == "rgb8" || m_srcEncording == "bgr8"){
            if (CheckNPP(nppiResize_8u_C3R((Npp8u *)*src, m_srcStep, m_srcSize, m_srcROI,
                                                (Npp8u *)*dst, m_dstStep, m_dstSize, m_dstROI,
                                                NPPI_INTER_LINEAR)) != 0)
                return false;

            return true;
        }
        else if (m_srcEncording == "rgb16" || m_srcEncording == "bgr16"){
            if (CheckNPP(nppiResize_16u_C3R((Npp16u *)*src, m_srcStep, m_srcSize, m_srcROI,
                                                 (Npp16u *)*dst, m_dstStep, m_dstSize, m_dstROI,
                                                 NPPI_INTER_LINEAR)) != 0)
                return false;

            return true;
        }
        else if (m_srcEncording == "rgba8" || m_srcEncording == "bgra8"){
            if (CheckNPP(nppiResize_8u_C4R((Npp8u *)*src, m_srcStep, m_srcSize, m_srcROI,
                                                (Npp8u *)*dst, m_dstStep, m_dstSize, m_dstROI,
                                                NPPI_INTER_LINEAR)) != 0)
                return false;

            return true;
        }
        else if (m_srcEncording == "rgba16" || m_srcEncording == "bgra16"){
            if (CheckNPP(nppiResize_16u_C4R((Npp16u *)*src, m_srcStep, m_srcSize, m_srcROI,
                                                 (Npp16u *)*dst, m_dstStep, m_dstSize, m_dstROI,
                                                 NPPI_INTER_LINEAR)) != 0)
                return false;

            return true;
        }
        else
            return false;
    };

private:
    NppiSize m_srcSize, m_dstSize;
    NppiRect m_srcROI, m_dstROI;
}; 
}

#endif