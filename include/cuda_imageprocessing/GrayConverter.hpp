#ifndef GRAY_CONVERTER_H
#define GRAY_CONVERTER_H

#include <vector>

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

namespace ips {

class GrayConverter : public ImageProcessing{
public:    
    GrayConverter(){
        coeffR = 0.299;
        coeffG = 0.587;
        coeffB = 0.114;
        coeffA = 1;   
    };

    bool m_Run(const sensor_msgs::Image &srcMsg, sensor_msgs::Image &dstMsg){
        if (!m_InitialMember(srcMsg))
            return false;

        std::vector<uint8_t> grayImage(m_dstStep * m_dstH, 0);

        {
            void *srcImage = nullptr, *dstImage = nullptr;

            if (!m_CudaAllocation(&srcImage, &dstImage)) return false;

            if (CheckCUDA(
                cudaMemcpy2D(srcImage, m_srcStep, 
                             &srcMsg.data[0], m_srcStep, 
                             m_srcW * m_srcChannelNum, m_srcH, cudaMemcpyHostToDevice), 
                "cudaMemcpy2D") != 0) return false;

            if (!m_Processing(&srcImage, &dstImage)) return false;

            if (CheckCUDA(
                cudaMemcpy2D(&grayImage[0], m_dstW * m_dstChannelNum, 
                                        dstImage, m_dstStep, 
                                        m_dstW * m_dstChannelNum, m_dstH, cudaMemcpyDeviceToHost),
                "cudaMemcpy2D") != 0) return false;

            nppiFree(srcImage);
            nppiFree(dstImage);
        }

        {
            dstMsg.header.frame_id = srcMsg.header.frame_id;
            dstMsg.width = m_dstW;
            dstMsg.height = m_dstH;
            dstMsg.step = m_dstStep;
            dstMsg.encoding = m_dstEncording;
            dstMsg.is_bigendian = srcMsg.is_bigendian;

            dstMsg.data = std::move(grayImage);
        }

        return true;        
    };

protected:
    bool m_InitialMember(const sensor_msgs::Image &srcMsg){
        if ((srcMsg.width < 0) || (srcMsg.height < 0)){
            ROS_ERROR("[GrayConverter] Unvalid image size. check your image.");
            return false;
        }

        if (srcMsg.encoding != "rgb8"   &&
            srcMsg.encoding != "rgb16"  &&
            srcMsg.encoding != "rgba8"  &&
            srcMsg.encoding != "rgba16" &&
            srcMsg.encoding != "bgr8"   &&
            srcMsg.encoding != "bgr16"  &&
            srcMsg.encoding != "bgra8"  &&
            srcMsg.encoding != "bgra16"){

            ROS_ERROR("[GrayConverter]%s is invalid encording format! Not supportted encording type.", srcMsg.encoding.c_str());
            return false;
        }

        m_srcW = srcMsg.width;
        m_srcH = srcMsg.height;
        m_dstW = srcMsg.width;
        m_dstH = srcMsg.height;
        m_size.width = srcMsg.width;
        m_size.height = srcMsg.height;

        m_srcEncording = srcMsg.encoding;
        m_srcChannelNum = sensor_msgs::image_encodings::numChannels(m_srcEncording);
        m_dstChannelNum = 1;

        if(sensor_msgs::image_encodings::bitDepth(m_srcEncording) == 8)
            m_dstEncording = sensor_msgs::image_encodings::MONO8;
        else
            m_dstEncording = sensor_msgs::image_encodings::MONO16;


        m_srcStep = m_srcW * m_srcChannelNum;
        m_dstStep = m_dstW * m_dstChannelNum;

        m_coeffs.push_back(coeffR);
        m_coeffs.push_back(coeffG);
        m_coeffs.push_back(coeffB);
        if(m_srcChannelNum == 4)
            m_coeffs.push_back(coeffA);
        

        return true;
    };
    
    bool m_CudaAllocation(void **src, void **dst){
        int srcStep, dstStep;

        if (m_srcEncording == "rgb8" || m_srcEncording == "bgr8"){
            *src = nppiMalloc_8u_C3(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_8u_C1(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else if (m_srcEncording == "rgb16" || m_srcEncording == "bgr16"){
            *src = nppiMalloc_16u_C3(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_16u_C1(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else if (m_srcEncording == "rgba8" || m_srcEncording == "bgra8"){
            *src = nppiMalloc_8u_C4(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_8u_C1(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else if (m_srcEncording == "rgba16" || m_srcEncording == "bgra16"){
            *src = nppiMalloc_16u_C4(m_srcW, m_srcH, &srcStep);
            *dst = nppiMalloc_16u_C1(m_dstW, m_dstH, &dstStep);
            return true;
        }
        else
            return false;
    };

    bool m_Processing(void **src, void **dst){
        if (m_srcEncording == "rgb8" || m_srcEncording == "bgr8"){
            if (CheckNPP(
                nppiColorToGray_8u_C3C1R((Npp8u *)*src, m_srcStep,
                                         (Npp8u *)*dst, m_dstStep,
                                         m_size, &m_coeffs[0]),
                "nppiColorToGray_8u_C3C1R") != 0) return false;

            return true;
        }
        else if (m_srcEncording == "rgb16" || m_srcEncording == "bgr16"){
            if (CheckNPP(
                nppiColorToGray_16u_C4C1R((Npp16u *)*src, m_srcStep,
                                          (Npp16u *)*dst, m_dstStep,
                                          m_size, &m_coeffs[0]),
                "nppiColorToGray_16u_C4C1R") != 0) return false;
    
            return true;
        }
        else if (m_srcEncording == "rgba8" || m_srcEncording == "bgra8"){
            if (CheckNPP(
                nppiColorToGray_8u_C3C1R((Npp8u *)*src, m_srcStep,
                                         (Npp8u *)*dst, m_dstStep,
                                         m_size, &m_coeffs[0]),
                "nppiColorToGray_8u_C3C1R") != 0) return false;
    
            return true;
        }
        else if (m_srcEncording == "rgba16" || m_srcEncording == "bgra16"){
            if (CheckNPP(
                nppiColorToGray_16u_C3C1R((Npp16u *)*src, m_srcStep,
                                           (Npp16u *)*dst, m_dstStep,
                                           m_size, &m_coeffs[0]),
                "nppiColorToGray_16u_C3C1R") != 0) return false;
    
            return true;
        }
        else
            return false;
    };

private:
    NppiSize m_size;
    std::vector<Npp32f> m_coeffs;
    
    float coeffR;
    float coeffG;
    float coeffB;
    float coeffA;
};

}

#endif
