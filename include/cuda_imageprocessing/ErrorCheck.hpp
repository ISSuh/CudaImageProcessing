#ifndef ERROR_CHECK_HPP
#define ERROR_CHECK_HPP

// ROS
#include <ros/ros.h>
#include <ros/console.h>

// CUDA
#include <npp.h>
#include <nvJPEG/nvjpeg.h>

namespace ips{

inline cudaError_t CheckCUDA(cudaError_t status){
	if (status != cudaSuccess)
            ROS_ERROR("[CUDA ERROR] %d", status);
        
        return status;
}

inline NppStatus CheckNPP(NppStatus status){
        if (status != NPP_NO_ERROR)
             ROS_ERROR("[NPP ERROR] %d", status);
        
        return status;
}

inline nvjpegStatus_t ChecknvJPEG(nvjpegStatus_t status){
	if (status != NVJPEG_STATUS_SUCCESS)
            ROS_ERROR("[CUDA ERROR] %d", status);
        
        return status;
}

}

#endif