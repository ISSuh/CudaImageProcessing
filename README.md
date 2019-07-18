# CudaImageProcessing

## ROS based image resize & compressed & togray scale using CUDA(NPP, nvJPEG)

----------------------------

### ***Development Enviroments***

>  - Ubuntu 16.04 LTS
>  - ROS Kinetic
>  - MX150 (Driver v410.78)
>  - CUDA v9.0
>  - NPP v9.0.225
>  - nvJPEG  v0.1.5

### ***Supported color channel***

> - Image Resize\
> -- Only Supported [RGB8, RGBA8, RGB16, RGBA16, BGR8, BGRA8, BGR16, BGRA16, MONO8, MONO16]
> - Image convert to gray scale\
> -- Only Supported [RGB8, RGBA8, RGB16, RGBA16, BGR8, BGRA8, BGR16, BGRA16]  
> - Image Compressed\
> -- Only Supported [RGB8, BGR8]  

### ***Caution!!***

> I tested Only my development enviroments.

### History

> update image compressed ("sensor_msgs/Image" to "sensor_msgs/CompressedImage")