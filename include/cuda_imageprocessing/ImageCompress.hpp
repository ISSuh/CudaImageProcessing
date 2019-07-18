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

namespace ips{

class ImageCompress : public ImageProcessing{
public:
    ImageCompress(int jpegQuality, int jpegHuffmanOprimzie)
        : quality(jpegQuality), huffmanOptimized(jpegHuffmanOprimzie) {}

    bool m_Run(const sensor_msgs::Image &srcMsg, sensor_msgs::CompressedImage &dstMsg) {
        if (!m_InitialMember(srcMsg))
            return false;

        {
            if (!m_CudaAllocation(NULL, NULL))
                return false;

            if (CheckCUDA(
                cudaMalloc((void **)&(nvImage.channel[0]), msgImageLen),
                "cudaMalloc") != 0)
                return false;

            if (CheckCUDA(
                cudaMemcpy(nvImage.channel[0], &srcMsg.data[0],
                            msgImageLen, cudaMemcpyHostToDevice),
                "cudaMemcpy") != 0)
                return false;

            if (!m_Processing(NULL, NULL))
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
            dstMsg.data = std::move(jpeg);
        }

        return true;
    };

protected:
    virtual bool m_InitialMember(const sensor_msgs::Image &srcMsg) {
        if ((srcMsg.width < 0) || (srcMsg.height < 0)) {
            ROS_ERROR("[ImageCompress] Unvalid image size. check your image.");
            return false;
        }

        if (srcMsg.encoding != "rgb8" &&
            srcMsg.encoding != "bgr8") {

            ROS_ERROR("[ImageCompress] %s is invalid encording format! Not supportted encording type.", srcMsg.encoding.c_str());
            return false;
        }
        else{
            if (srcMsg.encoding == "rgb8")
                inputFormat = NVJPEG_INPUT_RGBI;
            else if (srcMsg.encoding == "bgr8")
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

    virtual bool m_CudaAllocation(void **src, void **dst)
    {
        if (CheckCUDA(
            cudaStreamCreate(&stream),
            "cudaStreamCreate") != 0)
            return false;

        if (ChecknvJPEG(
            nvjpegCreateSimple(&handle),
            "nvjpegCreateSimple") != 0)
            return false;

        if (ChecknvJPEG(
            nvjpegEncoderStateCreate(handle, &encState, stream),
            "nvjpegEncoderStateCreate") != 0)
            return false;

        if (ChecknvJPEG(
            nvjpegEncoderParamsCreate(handle, &encParams, stream),
            "nvjpegEncoderParamsCreate") != 0)
            return false;

        if (ChecknvJPEG(
            nvjpegEncoderParamsSetQuality(encParams, quality, stream),
            "nvjpegEncoderParamsSetQuality") != 0)
            return false;

        if (ChecknvJPEG(
            nvjpegEncoderParamsSetOptimizedHuffman(encParams, huffmanOptimized, stream),
            "nvjpegEncoderParamsSetOptimizedHuffman") != 0)
            return false;

        if (ChecknvJPEG(
            nvjpegEncoderParamsSetSamplingFactors(encParams, NVJPEG_CSS_444, stream),
            "nvjpegEncoderParamsSetSamplingFactors") != 0)
            return false;

        return true;
    };

    virtual bool m_Processing(void **src, void **dst)
    {
        if (ChecknvJPEG(
            nvjpegEncodeImage(handle, encState, encParams,
                                &nvImage, inputFormat,
                                m_srcW, m_srcH, stream),
            "nvjpegEncodeImage") != 0)
            return false;

        if (ChecknvJPEG(
            nvjpegEncodeRetrieveBitstream(handle, encState, NULL, &buffLen, stream),
            "nvjpegEncodeRetrieveBitstream") != 0)
            return false;

        if (CheckCUDA(
            cudaStreamSynchronize(stream),
            "cudaStreamSynchronize") != 0)
            return false;

        jpeg.resize(buffLen);

        if (ChecknvJPEG(
            nvjpegEncodeRetrieveBitstream(handle, encState, jpeg.data(), &buffLen, 0),
            "nvjpegEncodeRetrieveBitstream") != 0)
            return false;

        if (CheckCUDA(
            cudaStreamSynchronize(stream),
            "cudaStreamSynchronize") != 0)
            return false;

        return true;
    };

private:
    nvjpegHandle_t handle;
    nvjpegEncoderState_t encState;
    nvjpegEncoderParams_t encParams;
    cudaStream_t stream;
    nvjpegImage_t nvImage;
    std::vector<uint8_t> jpeg;

    size_t msgImageLen;
    size_t buffLen;

    int quality;
    int huffmanOptimized;
    nvjpegInputFormat_t inputFormat;
};

} // namespace ips

#endif