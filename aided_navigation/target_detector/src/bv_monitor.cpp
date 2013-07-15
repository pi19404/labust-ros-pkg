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
#include <aidnav_msgs/MBSonar.h>
#include <sensor_msgs/image_encodings.h>
#include <aidnav_msgs/SetRange.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <exception>

ros::Time last;

cv::Mat disp(const cv::Mat& img, float max = 1024)
{
	cv::Mat nimg;
	img.convertTo(nimg,CV_32FC1);
	return (nimg=nimg/max);
}

void normalize(const cv::Mat& frame)
{
	cv::Scalar mean, std;
	double min,max;

	cv::Mat nimg = frame;
	//frame.convertTo(nimg,CV_32FC1);
	//nimg /= 65536;
	cv::meanStdDev(nimg,mean,std);
	//std::cout<<"Mean:"<<mean<<","<<std<<std::endl;
	nimg = (nimg - mean.val[0])*std.val[0];

	cv::minMaxLoc(nimg,&min,&max);
	//std::cout<<"Max:"<<max<<std::endl;

	//cv::imshow("Normalized",disp(nimg,512));
}

std::pair<float, float> noise_estimate(cv::Mat& meanM, cv::Mat& stdM, size_t x, size_t y, int colSpan, int rowSpan)
{
	std::pair<float, float> noisep;
	noisep.first = 0;
	noisep.second = 0;

	int nrois = 0;
	for (int i=-colSpan; i<colSpan; i+=colSpan)
	{
		for (int j=-rowSpan; j<rowSpan; j+=rowSpan)
		{
			if ((i!=0) || (j!=0))
			{
				int xroi = x+i;
				int yroi = y+j;

				if ((xroi<0) || (xroi>meanM.cols) || (yroi<0) || (yroi>meanM.rows)) continue;
				++nrois;
				noisep.first += meanM.at<float>(xroi,yroi);
				noisep.second += stdM.at<float>(xroi,yroi);
			}
		}
	}

	noisep.first /= nrois;
	noisep.second /= nrois;

	return noisep;
}

void callback(const aidnav_msgs::MBSonarConstPtr& image)
{
	ROS_INFO("Received image: %d x %d x 2 = %d",image->image.width, image->image.height,image->image.data.size());
	ROS_INFO("Elapsed time: %f",(ros::Time::now() - last).toSec());
	last = ros::Time::now();

	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(image->image, sensor_msgs::image_encodings::MONO16);

	cv::imshow("Original",disp(cv_ptr->image,512));

	cv::Mat org = cv_ptr->image.clone();

	//cv::imwrite("high_res.bmp",cv_ptr->image,);

	size_t colSpan = 40, rowSpan = 40;

	cv::Mat meanM(org.cols,org.rows,CV_32FC1);
	cv::Mat stdM(org.cols,org.rows,CV_32FC1);
  for(size_t i=colSpan/2; i<org.cols; i+=colSpan)
	{
		for (size_t j=rowSpan/2; j<org.rows; j+=rowSpan)
		{
			cv::Rect roi(i-colSpan/2,j-rowSpan/2,colSpan,rowSpan);
			cv::Mat roiImg(org,roi);
			//cv::rectangle(org,roi,cv::Scalar(255,2552,255),1);
			cv::Scalar mean, std;
			cv::meanStdDev(roiImg,mean,std);
			meanM.at<float>(i,j) = mean.val[0];
			stdM.at<float>(i,j) = std.val[0];
			//std::cout<<"The ROI mean:"<<noisep.first<<std::endl;
			//roiImg = roiImg - noisep.first;
			//normalize(roiImg);
		}
	}

  cv::imshow("Normalized",disp(org,512));

  org = cv_ptr->image.clone();
  cv::Mat org2;
  org.convertTo(org2, CV_32FC1);
  org = org2;

  for(size_t i=colSpan/2; i<org.cols; i+=colSpan)
	{
		for (size_t j=rowSpan/2; j<org.rows; j+=rowSpan)
		{
			cv::Rect roi(i-colSpan/2,j-rowSpan/2,colSpan,rowSpan);
			cv::Mat roiImg(org,roi);
			std::pair<float, float> noisep = noise_estimate(meanM, stdM, i,j, colSpan, rowSpan);
			std::cout<<"The ROI mean:"<<noisep.first<<std::endl;
			roiImg = (roiImg - noisep.first);
			normalize(roiImg);
			cv::rectangle(org,roi,cv::Scalar(255,2552,255),1);
		}
	}
  double min,max;
	cv::minMaxLoc(org,&min,&max);
  cv::imshow("Normalized2",disp(org,1));


  org = cv_ptr->image.clone();

  for(size_t i=colSpan/2; i<org.cols; i+=colSpan)
	{
		for (size_t j=rowSpan/2; j<org.rows; j+=rowSpan)
		{
			cv::Rect roi(i-colSpan/2,j-rowSpan/2,colSpan,rowSpan);
			cv::Mat roiImg(org,roi);
			//cv::rectangle(org,roi,cv::Scalar(255,2552,255),1);
			//std::pair<float, float> noisep = noise_estimate(meanM, stdM, i,j, colSpan, rowSpan);
			//std::cout<<"The ROI mean:"<<noisep.first<<std::endl;
			roiImg = (roiImg - meanM.at<float>(i,j))*stdM.at<float>(i,j);
			//normalize(roiImg);
		}
	}

  cv::imshow("Normalized3",disp(org,512));

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

void callback2(const aidnav_msgs::MBSonarConstPtr& image)
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



