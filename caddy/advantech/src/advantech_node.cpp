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
 *
 *  Author : Dula Nad
 *  Created: 23.07.2013.
 *********************************************************************/
#include <REL_Linux_SUSI.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <std_msgs/UInt16.h>

#include <ros/ros.h>
#include <boost/bind.hpp>

struct SharedData
{
	enum {tcpu,tsys,taux};
	uint16_t tempTypes;
	float temp[3];
	bool hasIO;
};

int init_susi(SharedData& shared)
{
	if (!SusiDllInit())
	{
		ROS_ERROR("Cannot initialize advantech SUSI drivers.");
		return 1;
	}

	bool hasHwm(SusiHWMAvailable());
	shared.tempTypes = 0;
	if (!hasHwm)
	{
		ROS_WARN("No HWM available");
	}
	else
	{
		float retVal;
		SusiHWMGetTemperature(0, &retVal, &shared.tempTypes);
	}

	shared.hasIO = SusiIOAvailable();
	if (!shared.hasIO)
	{
		ROS_WARN("No IO available");
	}
	else
	{
		//Put all pins as outgoing
		DWORD mask(0);
		DWORD pins(0x00FF);
		SusiIOSetDirectionMulti(pins,&mask);
	}

	return 0;
}

void getTemperature(SharedData& shared)
{
	if (TCPU & shared.tempTypes)
	{
		SusiHWMGetTemperature(TCPU, &shared.temp[SharedData::tcpu], NULL);
	}
	if (TSYS & shared.tempTypes)
	{
		SusiHWMGetTemperature(TSYS, &shared.temp[SharedData::tsys], NULL);
	}
	if (TAUX & shared.tempTypes)
	{
		SusiHWMGetTemperature(TSYS, &shared.temp[SharedData::taux], NULL);
	}
}

void handleGPIO(SharedData& shared, const std_msgs::UInt16::ConstPtr& data)
{
	if (shared.hasIO)
	{
		DWORD pins = data->data & 0xFF00 > 8;
		DWORD status = data->data & 0x00FF;
		SusiIOWriteMulti(pins, status);
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"advantech_io");

	SharedData shared;
  if (init_susi(shared))
  {
  	SusiDllUnInit();
  	return -1;
  };

	ros::NodeHandle nh;
	//Publishers
	ros::Publisher diagnostics = nh.advertise<diagnostic_msgs::DiagnosticStatus>("advantech_diagnostic",1);
	ros::Subscriber gpio = nh.subscribe<std_msgs::UInt16>("gpio", 1, boost::bind(&handleGPIO,boost::ref(shared),_1));

	ros::Rate rate(10);

	std::string temps[3] = {"TCPU","TSYS","TAUX"};
	diagnostic_msgs::DiagnosticStatus status;
	status.level = status.OK;
	status.name = "AdvantechBoard";
	status.message = "Temperature report";
	status.hardware_id = "AdvantechSUSI";
	for(int i=0;i<3;++i) status.values.push_back(diagnostic_msgs::KeyValue());

	while (ros::ok())
	{
		getTemperature(shared);

		//Publish temperature
		std::ostringstream out;
		for(int i=0;i<3;++i)
		{
			out.str("");
			out<<shared.temp[i];
			status.values[i].key = temps[i];
			status.values[i].value = out.str();
		}
		diagnostics.publish(status);

		rate.sleep();
		ros::spinOnce();
	}

	SusiDllUnInit();
	return 0;
}

