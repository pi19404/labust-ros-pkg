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
#include <sensor_msgs/Image.h>
#include <labust_bv/SetRange.h>
#include <sensor_msgs/image_encodings.h>

#include <bvt_sdk.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <exception>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

BVTHead configure_sonar(BVTSonar sonar, const ros::NodeHandle& phandle)
{
	std::string sonarType("FILE"),sonarAddress("swimmer.son");
	int sonarHead(0);

	phandle.param("Type",sonarType,sonarType);
	phandle.param("Address",sonarAddress,sonarAddress);
	phandle.param("SonarHead",sonarHead,sonarHead);

	if (int error = BVTSonar_Open(sonar,sonarType.c_str(),sonarAddress.c_str()))
	{
		throw std::invalid_argument(BVTError_GetString(error));
	}

	BVTHead head(0);
	if (int error = BVTSonar_GetHead(sonar,sonarHead,&head))
	{
		throw std::invalid_argument(BVTError_GetString(error));
	}

	return head;
}

boost::mutex mux;

bool setRange(BVTHead head, labust_bv::SetRange::Request& request, labust_bv::SetRange::Response& response)
{
	boost::mutex::scoped_lock l(mux);
	BVTHead_SetRange(head,request.start,request.stop);

	ROS_INFO("Setting range.");

	return true;
}

/*
 * Izmjene:
 *
 *  topics: bv_image - cijela slika (po zahtijevu)
 *          bv_roi_image_{id} - proizvoljni dio slike (možda da ima mogućnost za više njih)
 *          bv_roi_register - registracija za ROI slike
 *                            ROI poruka i ROI id (za slučaj da dodam više ROI-a)
 *
 *  servisi: akvizicija (po glavi) / pokretanje i sl.
 *           postavljanje udaljenosti
 *           paljenje logiranja
 *           kvaliteta beamforminga
 *
 *  Faktorirati funkcionalnost u labust_bv datoteku (acquistion manager)
 */

int main(int argc, char* argv[])
try
{
	//Setup ROS
	ros::init(argc, argv, "bv_node");
	ros::NodeHandle nhandle;

	//Configure this node
	std::string topicName("bv_image");
	int bufferSize(5),rate(10);
	ros::NodeHandle phandle("~");
	phandle.param("TopicName",topicName,topicName);
	phandle.param("BufferSize",bufferSize,bufferSize);
	phandle.param("Rate",rate,rate);
	phandle.getParam("PluginName",topicName);

	//Initialize sonar
	BVTSonar sonar = BVTSonar_Create();

	BVTHead head(0);
	try
	{
		head = configure_sonar(sonar, phandle);
	}
	catch (std::exception& e)
	{
		ROS_INFO("Exception during sonar configuration: %s",e.what());
		throw;
	}

	//Create topic publisher
	ros::Publisher imageTopic = nhandle.advertise<sensor_msgs::Image>(topicName,bufferSize);
	ros::Rate loop_rate(rate);

	//Offer the services
	ros::ServiceServer rangeService = nhandle.advertiseService<labust_bv::SetRange::Request, labust_bv::SetRange::Response>("SetRange",boost::bind(setRange,head,_1,_2));

	int pingNum = BVTHead_GetPingCount(head);
	int counter(0);

	BVTColorMapper mapper = BVTColorMapper_Create();
	BVTColorMapper_Load(mapper,"/home/dnad/Development/labust_svn/trunk/ros_labust/labust-ros-pkg/labust_bv/labust_bv_sdk/build/bvtsdk/colormaps/jet.cmap");
	BVTHead_SetImageRes(head,BVTHEAD_RES_AUTO);

	while (nhandle.ok())
	{
		ros::Time start=ros::Time::now();
		//Acquire ping
		BVTPing ping(0);
		boost::mutex::scoped_lock l(mux);
		if (int error = BVTHead_GetPing(head,counter%pingNum,&ping))
		{
			throw std::invalid_argument(BVTError_GetString(error));
		}
		l.unlock();

		//Acquire mag image
		BVTMagImage magImage(0);
		if (int error = BVTPing_GetImage(ping,&magImage))
		{
			throw std::invalid_argument(BVTError_GetString(error));
		}

		/*
		BVTColorImage cimg(0);
		BVTColorMapper_MapImage(mapper,magImage,&cimg);

		sensor_msgs::Image msg;
		msg.height = BVTColorImage_GetHeight(cimg);
		msg.width = BVTColorImage_GetWidth(cimg);
		const unsigned char* datap = reinterpret_cast<const unsigned char*>(BVTColorImage_GetBits(cimg));
		msg.encoding = "bgra8";
		msg.step = 4*msg.width;
		msg.step = 2*msg.width;
		msg.is_bigendian = 0;
		msg.data.assign(datap,datap + msg.step*msg.height);*/

		sensor_msgs::Image msg;
		msg.height = BVTMagImage_GetHeight(magImage);
		msg.width = BVTMagImage_GetWidth(magImage);
		const unsigned char* datap = reinterpret_cast<const unsigned char*>(BVTMagImage_GetBits(magImage));
		msg.encoding = sensor_msgs::image_encodings::MONO16;
		msg.step = 2*msg.width;
		msg.is_bigendian = 0;
		msg.data.assign(datap,datap + msg.step*msg.height);

		imageTopic.publish(msg);

	  ros::spinOnce();

	  //BVTColorImage_Destroy(cimg);
	  BVTMagImage_Destroy(magImage);
	  BVTPing_Destroy(ping);
		loop_rate.sleep();
		++counter;

		ROS_INFO("Elapsed time: %f",(ros::Time::now() - start).toSec());
	}

	BVTColorMapper_Destroy(mapper);
	BVTSonar_Destroy(sonar);

  return 0;
}
catch (const std::exception& e)
{
  std::cerr<<e.what()<<std::endl;
}
