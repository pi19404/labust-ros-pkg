/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, LABUST, UNIZG-FER
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the LABUST nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <bvt_sdk/MBSonar.h>
#include <sensor_msgs/image_encodings.h>
#include <bvt_sdk/SetRange.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <exception>

ros::Time last;

void normalize(const cv::Mat& frame)
{
	cv::Scalar mean, std;
	double min,max;

	cv::Mat nimg;// = (frame - mean.val[0])/std.val[0];
	frame.convertTo(nimg,CV_32FC1);
	nimg /= 4096;
	cv::meanStdDev(nimg,mean,std);
	std::cout<<"Mean:"<<mean<<","<<std<<std::endl;
	nimg = (nimg - mean.val[0])/std.val[0];

	cv::minMaxLoc(nimg,&min,&max);
	std::cout<<"Max:"<<max<<std::endl;

	cv::imshow("Normalized",nimg/max);


//    cv::Mat back;
//    cv::Mat fore;
//    cv::BackgroundSubtractorMOG2 bg;
//    //bg.set("nmixtures",3);
//    //bg.set("bShadowDetection",false);
//
//    std::vector<std::vector<cv::Point> > contours;
//
//    bg.operator ()(frame,fore);
//    bg.getBackgroundImage(back);
//    cv::erode(fore,fore,cv::Mat());
//    cv::dilate(fore,fore,cv::Mat());
//    cv::findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
//    cv::drawContours(frame,contours,-1,cv::Scalar(0,0,255),2);
//    cv::imshow("Frame",frame);
//    cv::imshow("Background",back);
}

void callback(const bvt_sdk::MBSonarConstPtr& image)
{
	ROS_INFO("Received image: %d x %d x 2 = %d",image->image.width, image->image.height,image->image.data.size());
	ROS_INFO("Elapsed time: %f",(ros::Time::now() - last).toSec());
	last = ros::Time::now();

	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(image->image, sensor_msgs::image_encodings::MONO16);

	cv::Mat nimg;// = (frame - mean.val[0])/std.val[0];
	cv_ptr->image.convertTo(nimg,CV_32FC1);
	nimg /= 4096;
	cv::imshow("Original",nimg);

	normalize(cv_ptr->image);

	cv::waitKey(10);


	/*ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<labust_bv::SetRange>("SetRange");
	labust_bv::SetRange service;

	service.request.stop = 15;
	service.request.start = 5;

	if (client.call(service))
	{
		ROS_INFO("Service ok.");
	}
	else
	{
		ROS_INFO("Service failed.");
	}
	*/
}

void callback2(const bvt_sdk::MBSonarConstPtr& image)
{
	ROS_INFO("Received image: %d x %d x 4 = %d",image->image.width, image->image.height,image->image.data.size());
	ROS_INFO("Elapsed time: %f",(ros::Time::now() - last).toSec());
	last = ros::Time::now();

	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(image->image, sensor_msgs::image_encodings::BGRA8);

	cv::imshow("SonarColor",cv_ptr->image);
	cv::waitKey(10);


	/*ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<labust_bv::SetRange>("SetRange");
	labust_bv::SetRange service;

	service.request.stop = 15;
	service.request.start = 5;

	if (client.call(service))
	{
		ROS_INFO("Service ok.");
	}
	else
	{
		ROS_INFO("Service failed.");
	}
	*/
}

int main(int argc, char* argv[])
try
{
	//Setup ROS
	ros::init(argc,argv,"bv_monitor");
	ros::NodeHandle nhandle;

	//Configure this node
	std::string topicName("bvsonar_img");
	std::string topicName2("bvsonar_cimg");
	int bufferSize(1),rate(10);
	ros::NodeHandle phandle("~");
	phandle.param("TopicName",topicName,topicName);
	phandle.param("BufferSize",bufferSize,bufferSize);
	phandle.param("Rate",rate,rate);

	//Create topic subscription
	ros::Subscriber imageTopic = nhandle.subscribe(topicName,bufferSize,callback);
	ros::Subscriber imageTopic2 = nhandle.subscribe(topicName2,bufferSize,callback2);

    cv::namedWindow("Original",0);
    cv::namedWindow("Normalized",0);

	ros::spin();
	return 0;
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
}



