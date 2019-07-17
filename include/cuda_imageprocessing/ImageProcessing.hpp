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
};

}
#endif