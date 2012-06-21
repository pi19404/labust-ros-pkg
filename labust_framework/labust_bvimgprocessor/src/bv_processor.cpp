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
#include <labust/blueview/TrackerROI.hpp>
#include <labust/blueview/ImageProcessing.hpp>
#include <labust/blueview/ProcessingChain.hpp>
#include <labust_bv/BVSonar.h>


#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/bind.hpp>

#include <exception>
#include <cmath>

struct click
{
	click():x(0),y(0),isNew(false){};
	int x,y;
	bool isNew;
};

click clickData;
click ROIPosition;

void mouse_cb(int event, int x, int y, int flags, void* param)
{
  switch( event )
  {
  case CV_EVENT_RBUTTONDOWN:
  {
    std::cout<<"Target: ("<<x<<", "<<y<<")\n";
    break;
  }
  case CV_EVENT_LBUTTONDOWN:
  {
  	/*
    BVSonar& son = *((BVSonar*)param);
    head.range = BVTMagImage_GetPixelRange(son.magImage(headNum),y,x);
    head.bearing = BVTMagImage_GetPixelRelativeBearing(son.magImage(headNum),y,x);
    processor.setPosition(head);*/
    std::cout<<"Vehicle on: ("<<x<<", "<<y<<")\n";
    clickData.x = x;
    clickData.y = y;
    clickData.isNew = true;
    //init = false;
    break;
  }
  default:
    break;
  }
}

void callback(labust::blueview::BVImageProcessor* /*labust::blueview::ProcessingChain<>*/ processor, const labust_bv::BVSonarConstPtr& image)
{
	//Do some time calculation for performance.
	static ros::Time last = ros::Time::now();
	//ROS_INFO("Received image: %d x %d x 2 = %d",image->image.width, image->image.height,image->image.data.size());
	//ROS_INFO("Elapsed time: %f",(ros::Time::now() - last).toSec());
	last = ros::Time::now();
	//Get the magnitude image
	labust::blueview::TrackerROI roi;
	roi.origin.x = roi.origin.y = 0;
	roi.headData.bearing = roi.headData.heading = roi.headData.range = 0;
	roi.headData.tiltAngle = roi.headData.panAngle = 0;
	roi.headData.resolution = image->resolution;

	if (clickData.isNew)
	{
		clickData.isNew = false;
		float x = -(clickData.x - image->origin.x)*image->resolution;
		float y = -(clickData.y - image->origin.y)*image->resolution;
    roi.headData.range = std::sqrt(std::pow(x,2) + std::pow(y,2));
    roi.headData.bearing =std::atan2(x,y);
    //processor.setPosition(head);
    std::cout<<"Origin ("<<image->origin.x<<","<<image->origin.y<<")"<<std::endl;
    std::cout<<"Range:"<<roi.headData.range<<","<<"bearing:"<<roi.headData.bearing<<std::endl;

    processor->setPosition(roi.headData);
    ROIPosition = clickData;
	}

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image->image, sensor_msgs::image_encodings::MONO16);

	if (processor->isTracking())
	{
		int roi_len = 4/image->resolution;

		//std::cout<<"Feature:("<<ROIPosition.x<<","<<ROIPosition.y<<")"<<std::endl;
		cv::Point roiup(ROIPosition.x - roi_len/2, ROIPosition.y - roi_len/2);

		int xrest = cv_ptr->image.size().width - (roiup.x + roi_len);
		int yrest = cv_ptr->image.size().height - (roiup.y + roi_len);

		if ((xrest<0) || (yrest<0)) std::cout<<"Out of bounds."<<std::endl;

		cv::Rect rect(roiup.x, roiup.y, (xrest<0)?(roi_len+xrest):roi_len, (yrest<0)?(roi_len+yrest):roi_len);

		roi.roi = cv::Mat(cv_ptr->image, rect);
		roi.size = roi.roi.size();
    roi.origin.y = -(roiup.y - image->origin.y);
    roi.origin.x = -(roiup.x - image->origin.x);
		//labust::blueview::TrackedFeaturePtr feature = processor->processROI(roi);
    //Draw ROI
    cv::rectangle(cv_ptr->image, rect, cv::Scalar(255,255,255), 3);

    //processor->processROI(roi);
    if (processor->processROI(roi))
    {
    	std::cout<<"Found contact."<<std::endl;
    	labust::blueview::TrackedFeature feature = processor->getTracklet();
    	//ROIPosition.y = roiup.y + feature->pposition.y;
    	//ROIPosition.x = roiup.x + feature->pposition.x;
    	ROIPosition.y = roiup.y + feature.pposition.y;
    	ROIPosition.x = roiup.x + feature.pposition.x;
    }
    //else
    {
    	std::cout<<"Unable to find contact."<<std::endl;
    }
	}

	cv::imshow("Sonar",cv_ptr->image*200);
	cv::waitKey(10);
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

	labust::blueview::BVImageProcessor processor;
	//labust::blueview::ProcessingChain<> processor;

	//Create a window
	cv::namedWindow("Sonar",0);
	cv::setMouseCallback("Sonar",&mouse_cb,0);

	//Create topic subscription
	ros::Subscriber imageTopic = nhandle.subscribe<labust_bv::BVSonar>(topicName,bufferSize,boost::bind(&callback,&processor,_1));

	ros::spin();
	return 0;
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
}



