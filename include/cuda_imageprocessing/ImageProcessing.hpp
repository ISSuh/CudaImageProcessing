#ifndef IMAGE_PROCESSING_HPP
#define IMAGE_PROCESSING_HPP

#include <sensor_msgs/Image.h>
#include <boost/shared_ptr.hpp>

namespace ips {

class ImageProcessing{
public: 
    virtual ~ImageProcessing() { }
    bool Run();

protected:
    virtual bool InitialMember(const sensor_msgs::Image &srcMsg) = 0;
    virtual bool CudaAllocation(void **src, void **dst) = 0;
    virtual bool Processing(void **src, void **dst) = 0;

    int m_srcW, m_srcH;
    int m_dstW, m_dstH;
    int m_srcStep, m_dstStep;

    std::string m_srcEncording;
    std::string m_dstEncording;

    uint32_t m_srcChannelNum;
    uint32_t m_dstChannelNum;
};

}
#endif