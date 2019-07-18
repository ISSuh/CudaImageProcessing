#ifndef ERROR_CHECK_HPP
#define ERROR_CHECK_HPP

// ROS
#include <ros/ros.h>
#include <ros/console.h>

// CUDA
#include <npp.h>
#include <nvJPEG/nvjpeg.h>

namespace ips {

inline cudaError_t CheckCUDA(cudaError_t status, const std::string func) {
        if (status != cudaSuccess)
                ROS_ERROR("[CUDA ERROR][%s] %d", func.c_str(), status);

        return status;
}

inline NppStatus CheckNPP(NppStatus status, const std::string func) {
        if (status != NPP_NO_ERROR)
                ROS_ERROR("[NPP ERROR][%s] %d", func.c_str(), status);

        return status;
}

inline nvjpegStatus_t ChecknvJPEG(nvjpegStatus_t status, const std::string func) {
        if (status != NVJPEG_STATUS_SUCCESS)
                ROS_ERROR("[nvJPEG ERROR][%s] %d", func.c_str(), status);

        return status;
}

} // namespace ips

#endif