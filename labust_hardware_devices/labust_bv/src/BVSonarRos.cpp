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
#include <labust_bv/BVSonar.h>
#include <sensor_msgs/image_encodings.h>

using namespace labust::blueview;

BVSonarRos::BVSonarRos():
		phandle("~"),
		sonar(BVFactory::makeBVSonar()),
		pingRate(100)
{
	this->configure();
}

BVSonarRos::~BVSonarRos(){};

void BVSonarRos::configure()
{
	std::string sonarType("FILE");
	std::string sonarAddress("swimmer.son");
	std::string magtopicName("bvsonar_img");
	std::string colortopicName("bvsonar_cimg");
	std::string colormap("jet");
	std::string colormapDir(".");
	int headNumber(0);

	phandle.param("Type",sonarType,sonarType);
	phandle.param("Address",sonarAddress,sonarAddress);
	phandle.param("HeadNumber",headNumber,headNumber);
	phandle.param("MagnitudeTopicName",magtopicName,magtopicName);
	phandle.param("ColorTopicName",colortopicName,colortopicName);
	phandle.param("Colormap",colormap,colormap);
	phandle.param("ColormapDir",colormapDir,colormapDir);
	phandle.param("PingRate",pingRate,pingRate);

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

	//Topics configuration
	imageTopic = nhandle.advertise<labust_bv::BVSonar>(magtopicName,1);
	cImageTopic = nhandle.advertise<labust_bv::BVSonar>(colortopicName,1);
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
	while (nhandle.ok())
	{
		BVPingPtr ping(BVFactory::getBVPing(head,counter));
		BVMagImagePtr image(BVFactory::getBVMagImage(ping));
		BVColorImagePtr cimage(BVFactory::getBVColorImage(image,mapper));

		labust_bv::BVSonarPtr msg(new labust_bv::BVSonar());
		msg->image.height = BVTMagImage_GetHeight(image.get());
		msg->image.width = BVTMagImage_GetWidth(image.get());
		msg->image.encoding = sensor_msgs::image_encodings::MONO16;
		msg->image.step = 2*msg->image.width;
		msg->image.is_bigendian = 0;

		unsigned char* datap = reinterpret_cast<unsigned char*>(BVTMagImage_GetBits(image.get()));
		msg->image.data.assign(datap,datap + msg->image.step*msg->image.height);
		imageTopic.publish(msg);

		/*
		msg.reset(new labust_bv::BVSonar());
		msg->image.height = BVTColorImage_GetHeight(cimage.get());
		msg->image.width = BVTColorImage_GetWidth(cimage.get());
		msg->image.encoding = sensor_msgs::image_encodings::BGRA8;
		msg->image.step = 4*msg->image.width;
		datap = reinterpret_cast<unsigned char*>(BVTColorImage_GetBits(cimage.get()));
		msg->image.data.assign(datap,datap + msg->image.step*msg->image.height);
		cImageTopic.publish(msg);
		*/

		++counter;
		counter %= pingNum;

		ros::spinOnce();
		loop_rate.sleep();
	}
}



