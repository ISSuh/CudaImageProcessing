// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <cuda_imageprocessing/ImageResize.hpp>
#include <cuda_imageprocessing/GrayConverter.hpp>
#include <cuda_imageprocessing/ImageCompress.hpp>

class ImageProcessingSample{
public:
	ImageProcessingSample(){
		n = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

		n->getParam("subTopic", subTopic);
		n->getParam("resize", runResize);
    	n->getParam("togray", runTogray);
    	n->getParam("compress", runCompress);
    	n->getParam("dstWidth", dstWidth);
    	n->getParam("dstHeight", dstHeight);
		n->getParam("jpegQuality", jpegQuality);
    	n->getParam("jpegHuffmanOptimize", jpegHuffmanOptimize);

		pubTopic = subTopic + "/";
		
		// ips::ImageResizer Construct
		// param : resize image width
		// param : resize image height
		if(runResize){
			resizer = std::unique_ptr<ips::ImageResize>(new ips::ImageResize(dstWidth, dstHeight));
			pubTopic += "resize";
		}
		// ips::GrayConverter Construct
		if(runTogray){
			toGray = std::unique_ptr<ips::GrayConverter>(new ips::GrayConverter());
			pubTopic += "/gray";
		}

		if(runCompress){
			compressor = std::unique_ptr<ips::ImageCompress>(new ips::ImageCompress(jpegQuality,jpegHuffmanOptimize));
			pubTopic += "/compressed";
		}

		sub = n->subscribe(subTopic, 10, &ImageProcessingSample::callback, this);

		if(runCompress)
			pub = n->advertise<sensor_msgs::CompressedImage>(pubTopic, 1);
		else
			pub = n->advertise<sensor_msgs::Image>(pubTopic, 1);

		ROS_INFO("CUDA Image Processing RUN");
	}

	void callback(const sensor_msgs::Image::ConstPtr &msg){
		// ips::ImageResize
		// param : Subscribed Image message
		// param : destination resized image
		if(runResize){
			if(!resizer->m_Run(*msg, resiezedImage)) return;
		}
		else
			resiezedImage = *msg;
		
		// ips::GrayConverter
		// param : Subscribed Image message
		// param : destination converted image
		if(runTogray){
			if(!toGray->m_Run(resiezedImage, grayImage)) return;
		}
		else
			grayImage = resiezedImage;

		// ips::ImageCompress
		// param : Subscribed Image message
		// param : destination converted image
		if(runCompress){
			if(!compressor->m_Run(grayImage, convertedCompImage)) return;
		}
		else
			convertedImage = grayImage;

		runCompress ? pub.publish(convertedCompImage) : pub.publish(convertedImage);
	}

private:
	std::unique_ptr<ros::NodeHandle> n;
	ros::Publisher pub;
	ros::Subscriber sub;
	
    std::unique_ptr<ips::ImageResize> resizer;
	std::unique_ptr<ips::GrayConverter> toGray;
	std::unique_ptr<ips::ImageCompress> compressor;

	sensor_msgs::Image resiezedImage;
	sensor_msgs::Image grayImage;
	sensor_msgs::Image convertedImage;
	sensor_msgs::CompressedImage convertedCompImage;

	// ROS params
	std::string subTopic;
	std::string pubTopic;
	bool runResize;
	bool runTogray;
	bool runCompress;
	int dstWidth;
	int dstHeight;
	int jpegQuality;
	int jpegHuffmanOptimize;

	bool isMono8;
};


int main(int argc, char **argv){
	ros::init(argc, argv, "ImageProvessing");	

	ImageProcessingSample sample;

	ros::spin();
}