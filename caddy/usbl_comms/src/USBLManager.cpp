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
*  Author: Dula Nad
*  Created: 02.04.2013.
*********************************************************************/
#include <labust/tritech/USBLManager.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(usbl,USBLManager,labust::tritech::USBLManager, nodelet::Nodelet)

using namespace labust::tritech;

USBLManager::USBLManager():
	state(idle),
	validNav(false){};

USBLManager::~USBLManager(){};

void USBLManager::onInit()
{
	ros::NodeHandle nh = this->getNodeHandle();
	navData = nh.subscribe<auv_msgs::NavSts>("usblFiltered",	1, boost::bind(&USBLManager::onNavMsg,this,_1));
	incoming = nh.subscribe<std_msgs::String>("incoming_data",	1, boost::bind(&USBLManager::onIncomingMsg,this,_1));
	outgoing = nh.advertise<std_msgs::String>("outgoing_data",1);
	auto_mode = nh.advertise<std_msgs::Bool>("auto_mode",1);

	worker = boost::thread(boost::bind(&USBLManager::run,this));
}

void USBLManager::run()
{
	while (ros::ok())
	{
		switch (state)
		{
		case idle: break;
		case waitForReply: break;
		case initDiver:
			//Check if position is known, assemble the init message
			if (validNav)
			{
				std_msgs::Bool data;
				data.data = false;
				auto_mode.publish(data);



				//Change to transmission state
				state = transmission;
			}
			break;
		case transmission:
			//Send last diver clean position update
		default:
			ROS_ERROR("Unknown state.");
			state = idle;
		}
	}
}

void USBLManager::onNavMsg(const auv_msgs::NavSts::ConstPtr nav)
{
	NODELET_DEBUG("Received nav message.");
//	encoder.latitude = nav->global_position.latitude;
//	encoder.longitude = nav->global_position.longitude;
//	encoder.z = nav->position.depth;
//
//	//Here we select based on the algorithm
//	uint64_t msg = encoder.pack<DiverMsg::PositionInit>();
//	char* p = reinterpret_cast<char*>(&msg);
//	std_msgs::StringPtr out(new std_msgs::String());
//	out->data.assign(p,sizeof(uint64_t));
//	outgoing.publish(out);
}

void USBLManager::onIncomingMsg(const std_msgs::String::ConstPtr msg)
{
	NODELET_DEBUG("Received modem message.");
//	if (msg->data.size() > 8)
//	{
//		std::string data(msg->data);
//		uint64_t* binData = reinterpret_cast<uint64_t*>(&data[0]);
//		encoder.unpack<DiverMsg::AutoDiver>(*binData);
//
//		NODELET_INFO("Recevied message: %d", encoder.msgType);
//	}
}
