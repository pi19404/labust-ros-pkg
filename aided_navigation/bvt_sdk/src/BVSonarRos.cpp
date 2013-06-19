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
#include <labust/blueview/BVSonarRos.hpp>
#include <aidnav_msgs/MBSonar.h>
#include <sensor_msgs/image_encodings.h>
#include <pluginlib/class_list_macros.h>

using namespace labust::blueview;

PLUGINLIB_DECLARE_CLASS(bvt_sdk,BVSonarRos,labust::blueview::BVSonarRos, nodelet::Nodelet)

BVSonarRos::BVSonarRos():
		sonar(BVFactory::makeBVSonar()),
		pingRate(100)
{}

BVSonarRos::~BVSonarRos()
{
	worker.join();
};

void BVSonarRos::onInit()
{
	phandle = this->getPrivateNodeHandle();
	nhandle = this->getNodeHandle();

	std::string sonarType("FILE");
	std::string sonarAddress("swimmer.son");
	std::string magtopicName("bvsonar_img");
	std::string colortopicName("bvsonar_cimg");
	std::string colormap("jet");
	std::string colormapDir(".");
	int headNumber(0), soundSpeed(1475);

	phandle.param("Type",sonarType,sonarType);
	phandle.param("Address",sonarAddress,sonarAddress);
	phandle.param("HeadNumber",headNumber,headNumber);
	phandle.param("MagnitudeTopicName",magtopicName,magtopicName);
	phandle.param("ColorTopicName",colortopicName,colortopicName);
	phandle.param("Colormap",colormap,colormap);
	phandle.param("ColormapDir",colormapDir,colormapDir);
	phandle.param("PingRate",pingRate,pingRate);
	phandle.param("SoundSpeed", soundSpeed, soundSpeed);

	//Sonar configuration
	sonar = BVFactory::makeBVSonar();
	mapper = BVFactory::makeBVColorMapper(colormapDir + "/" + colormap + ".cmap");

	if (int error = BVTSonar_Open(sonar.get(),sonarType.c_str(),sonarAddress.c_str()))
	{
		throw std::invalid_argument(BVTError_GetString(error));
	}
	if (int error = BVTSonar_GetHead(sonar.get(),headNumber,&head))
	{
		throw std::invalid_argument(BVTError_GetString(error));
	}
	//Sonar calibration
	BVTHead_SetSoundSpeed(head,soundSpeed);

	//Topics configuration
	imageTopic = nhandle.advertise<aidnav_msgs::MBSonar>(magtopicName,1);
	cImageTopic = nhandle.advertise<aidnav_msgs::MBSonar>(colortopicName,1);

	//Configure the dynamic reconfigure server
  server.setCallback(boost::bind(&BVSonarRos::dynrec, this, _1, _2));

  worker=boost::thread(boost::bind(&BVSonarRos::run,this));
}

void BVSonarRos::dynrec(bvt_sdk::BVSonarConfig& config, uint32_t level)
{
	this->config = config;
}

void BVSonarRos::run()
{
	this->runFileAcquisition();
}

void BVSonarRos::runFileAcquisition()
{
	size_t pingNum = BVTHead_GetPingCount(head);
	size_t counter = 0;
	ros::Rate loop_rate(pingRate);
	BVTHead_SetImageRes(head,BVTHEAD_RES_HIGH);
	BVTHead_SetImageType(head,BVTHEAD_IMAGE_RTHETA);
	this->config.double_param = BVTHead_GetMaximumRange(head);
	server.setConfigMax(config);
	this->config.double_param = BVTHead_GetStopRange(head);
  server.setConfigDefault(this->config);
	while (nhandle.ok())
	{
		BVTHead_SetRange(head,0,this->config.double_param);
		BVPingPtr ping(BVFactory::getBVPing(head,counter));

		BVMagImagePtr image(BVFactory::getBVMagImage(ping));
		BVColorImagePtr cimage(BVFactory::getBVColorImage(image,mapper));

		//Add navigation data reading.
		aidnav_msgs::MBSonarPtr msg(new aidnav_msgs::MBSonar());
		msg->image.height = BVTMagImage_GetHeight(image.get());
		msg->image.width = BVTMagImage_GetWidth(image.get());
		msg->image.encoding = sensor_msgs::image_encodings::MONO16;
		msg->image.step = 2*msg->image.width;
		msg->image.is_bigendian = 0;
		msg->range_res = BVTMagImage_GetRangeResolution(image.get());
		msg->origin.z = 0;
		msg->origin.x = BVTMagImage_GetWidth(image.get())/2;
		msg->origin.y = BVTMagImage_GetOriginRow(image.get());

		unsigned char* datap = reinterpret_cast<unsigned char*>(BVTMagImage_GetBits(image.get()));
		msg->image.data.assign(datap,datap + msg->image.step*msg->image.height);
		imageTopic.publish(msg);

		msg.reset(new aidnav_msgs::MBSonar());
		msg->image.height = BVTColorImage_GetHeight(cimage.get());
		msg->image.width = BVTColorImage_GetWidth(cimage.get());
		msg->image.encoding = sensor_msgs::image_encodings::BGRA8;
		msg->image.step = 4*msg->image.width;
		datap = reinterpret_cast<unsigned char*>(BVTColorImage_GetBits(cimage.get()));
		msg->image.data.assign(datap,datap + msg->image.step*msg->image.height);
		cImageTopic.publish(msg);

		++counter;
		counter %= pingNum;

		ros::spinOnce();
		loop_rate.sleep();
	}
}



