#ifndef IMAGE_COMPRESS_HPP
#define IMAGE_COMPRESS_HPP

#include <vector>

// interface
#include "ImageProcessing.hpp"

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

// CUDA NPP
#include <npp.h>
#include <nvJPEG/nvjpeg.h>

// Error Check
#include <cuda_imageprocessing/ErrorCheck.hpp>

namespace ips {

class ImageCompress : public ImageProcessing{
public: 
    ImageCompress(int jpegQuality, int jpegHuffmanOprimzie) 
    : quality(jpegQuality), huffmanOptimized(jpegHuffmanOprimzie) {}
    
    bool Run(const sensor_msgs::Image &srcMsg, sensor_msgs::CompressedImage &dstMsg){
        if (!InitialMember(srcMsg))
            return false;

        {
            void *dstImage = nullptr;

            if (!CudaAllocation(NULL, NULL))
                return false;

            if(CheckCUDA(cudaMalloc((void **)&(nvImage.channel[0]),msgImageLen)) != 0)
                return false;

    		if(CheckCUDA(cudaMemcpy(nvImage.channel[0], &srcMsg.data[0], msgImageLen, cudaMemcpyHostToDevice)) != 0)
                return false;

            if (!Processing(NULL, &dstImage))
                return false;

            nvjpegEncoderParamsDestroy(encParams);
		    nvjpegEncoderStateDestroy(encState);
		    cudaStreamDestroy(stream);
		    nvjpegDestroy(handle);
            cudaFree(nvImage.channel[0]);
        }

        // TODO Set data copy in Run func
        {   
            dstMsg.header.frame_id = srcMsg.header.frame_id;
            dstMsg.format = m_dstEncording;
            // dstMsg.data = std::move(dstImage);
        }

    };

protected:
    virtual bool InitialMember(const sensor_msgs::Image &srcMsg){
        if ((srcMsg.width <= 0) || (srcMsg.height <= 0)){
            ROS_ERROR("Unvalid image size. check your image.");
            return false;
        }

        if (srcMsg.encoding != "rgb8"   &&
            srcMsg.encoding != "rgb16"  &&
            srcMsg.encoding != "bgr8"   &&
            srcMsg.encoding != "bgr16"  ){

            ROS_ERROR("Invalid Encording value! Not supportted Encording Type.");
            return false;
        }
        else{
            if(srcMsg.encoding == "rgb8" || srcMsg.encoding == "rgb16")
                inputFormat = NVJPEG_INPUT_RGBI;
            else if(srcMsg.encoding == "bgr8" || srcMsg.encoding == "bgr16")
                inputFormat = NVJPEG_INPUT_BGRI;
        }

        m_srcW = srcMsg.width;
        m_srcH = srcMsg.height;
        m_dstW = srcMsg.width;
        m_dstH = srcMsg.height;

        m_srcEncording = srcMsg.encoding;
        m_dstEncording = srcMsg.encoding + "; jpeg compressed";
        m_srcChannelNum = 3;
        m_dstChannelNum = 3;

        m_srcStep = m_srcW * m_srcChannelNum;
        m_dstStep = m_dstW * m_dstChannelNum;

        msgImageLen = srcMsg.data.size();

        nvImage.pitch[0] = m_srcStep;	

        return true;
    };
    
    virtual bool CudaAllocation(void **src, void **dst){
		nvjpegDevAllocator_t dev_allocator = {&ImageCompress::DevMalloc, &ImageCompress::DevFree};

        if(CheckCUDA(cudaStreamCreate(&stream)) != 0)
            return false;

		if(ChecknvJPEG(nvjpegCreateSimple(&handle)) != 0);
            return false;

		if(ChecknvJPEG(nvjpegEncoderStateCreate(handle, &encState, stream)) !=0)
            return false;

		if(ChecknvJPEG(nvjpegEncoderParamsCreate(handle, &encParams, stream)) !=0)
            return false;

		if(ChecknvJPEG(nvjpegEncoderParamsSetQuality(encParams, quality, stream)) !=0)
            return false;

		if(ChecknvJPEG(nvjpegEncoderParamsSetOptimizedHuffman(encParams, huffmanOptimized, stream)) !=0)
            return false;

	    if(ChecknvJPEG(nvjpegEncoderParamsSetSamplingFactors(encParams, NVJPEG_CSS_444, stream)) !=0)
            return false;
    };

    virtual bool Processing(void **src, void **dst){
		std::vector<uint8_t> jpeg(buffLen, 0);

        if(ChecknvJPEG(nvjpegEncodeImage(handle, encState, encParams, &nvImage, inputFormat, m_srcW, m_srcH, stream)) != 0);
			return false;

		if(ChecknvJPEG(nvjpegEncodeRetrieveBitstream(handle, encState, NULL, &buffLen, stream)) != 0)
            return false;

		if(CheckCUDA(cudaStreamSynchronize(stream)) != 0)
            return false;

		if(ChecknvJPEG(nvjpegEncodeRetrieveBitstream(handle, encState, jpeg.data(), &buffLen, 0)) != 0)
            return false;
		
        if(CheckCUDA(cudaStreamSynchronize(stream)) != 0);
            return false;

        // TODO Set data copy in Processing func
		// compImage.data  = std::move(jpeg);
		*dst = &jpeg[0];
    };

private:
    static int DevMalloc(void** p, size_t s){
        return (int)cudaMalloc(p, s);
    }

    static int DevFree(void* p){
        return (int)cudaFree(p);
    }

    nvjpegHandle_t handle; 
	nvjpegEncoderState_t encState;
    nvjpegEncoderParams_t encParams;
    cudaStream_t stream;
	nvjpegImage_t nvImage;

    size_t msgImageLen;
	size_t buffLen;

    int quality;
    int huffmanOptimized;
    nvjpegInputFormat_t inputFormat; 
};

}

#endif