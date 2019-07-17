// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <cuda_imageprocessing/ImageResize.hpp>
#include <cuda_imageprocessing/GrayConverter.hpp>

class IP_Sample{
public:
	IP_Sample(){
		n_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());

		n_->getParam("subTopic", subTopic);
		n_->getParam("resize", runResize);
    	n_->getParam("togray", runTogray);
    	n_->getParam("compress", runCompress);
    	n_->getParam("dstWidth", dstWidth);
    	n_->getParam("dstHeight", dstHeight);

		pubTopic = subTopic + "/";

		//TEST
		// runResize = true;
		// runTogray = true;
		// dstWidth = 600;
		// dstHeight = 300;

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

		sub_ = n_->subscribe(subTopic, 10, &IP_Sample::callback, this);
		pub_ = n_->advertise<sensor_msgs::Image>(pubTopic, 1);

		ROS_INFO("CUDA Image Processing RUN");
	}

	void callback(const sensor_msgs::Image::ConstPtr &msg){
		sensor_msgs::Image resiezedImage;
		sensor_msgs::Image grayImage;
		sensor_msgs::Image convertedImage;

		// ips::ImageResize
		// param : Subscribed Image message
		// param : Resize & Publish Image message
		if(runResize){
			if(!resizer->Run(*msg, resiezedImage))
				return;
		}
		else
			resiezedImage = std::move(*msg);
		
		// ips::GrayConverter
		// param : Subscribed Image message
		// param : togray & Publish Image message
		// if(!toGray->Run(msg, convertedImage))
		if(runTogray){
			if(!toGray->Run(resiezedImage, grayImage))
				return;
		}
		else
			grayImage = std::move(resiezedImage);
		
		// ips::ImageCompress
		if(runCompress){
			return;
		}
		else
			convertedImage = std::move(grayImage);
		
		pub_.publish(convertedImage);
	}

private:
	std::unique_ptr<ros::NodeHandle> n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	
    std::unique_ptr<ips::ImageResize> resizer;
	std::unique_ptr<ips::GrayConverter> toGray;

	std::string subTopic;
	std::string pubTopic;
	bool runResize;
	bool runTogray;
	bool runCompress;

	int dstWidth;
	int dstHeight;

	bool isMono8;
};


int main(int argc, char **argv){
	ros::init(argc, argv, "ImageProvessing");	

	IP_Sample sample;

	ros::spin();
}