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
 *  Created: 14.02.2013.
 *********************************************************************/
#include <labust/tritech/MTDevice.hpp>
#include <labust/tritech/mtMessages.hpp>
#include <labust/tritech/mmcMessages.hpp>

#include <geometry_msgs/PointStamped.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <queue>

///\todo Join the USBLSim and real USBL classes ??
class USBLSim
{
public:
	USBLSim():
		usblBusy(false),
		useDevice(false),
		sent(false),
		navTime(4)
	{
		ros::NodeHandle nh,ph("~");
		std::string port("/dev/rfcomm0");
		int baud(57600);

		ph.param("port",port,port);
		ph.param("baud",baud,baud);
		ph.param("use_device",useDevice,useDevice);
		ph.param("nav_time",navTime,navTime);

		if (useDevice)
		{
			using namespace labust::tritech;
			usbl.reset(new MTDevice(port,baud));
			MTDevice::HandlerMap map;
			map[MTMsg::mtMiniModemCmd] = boost::bind(&USBLSim::onReplyMsg,this, _1);
			usbl->registerHandlers(map);
		}

		dataSub = nh.subscribe<std_msgs::String>("outgoing_data",	1, boost::bind(&USBLSim::onOutgoingMsg,this,_1));
		dataPub = nh.advertise<std_msgs::String>("incoming_data",1);
		usblTimeout = nh.advertise<std_msgs::Bool>("usbl_timeout",1);
		usblNav = nh.advertise<geometry_msgs::PointStamped>("usbl_nav", 1);
		simDiverState = nh.subscribe<auv_msgs::NavSts>("diver_sim",1, boost::bind(&USBLSim::onDiverSim, this, _1));
		simVehicleState = nh.subscribe<auv_msgs::NavSts>("platform_sim",1, boost::bind(&USBLSim::onPlatformSim, this, _1));
	}

	void onReplyMsg(labust::tritech::MTMsgPtr tmsg)
	{
		using namespace labust::tritech;
		{
			boost::mutex::scoped_lock lock(pingLock);
			usblBusy = false;
		}
		usblCondition.notify_one();

		boost::archive::binary_iarchive dataSer(*tmsg->data, boost::archive::no_header);
		MMCMsg modem_data;
		dataSer>>modem_data;
		//std::cout<<"Modem data:";
		//for(int i=0; i<modem_data.data.size(); ++i) std::cout<<int(modem_data.data[i])<<",";
		//std::cout<<std::endl;
		//std_msgs::String::Ptr modem(new std_msgs::String());
		//modem->data.assign(modem_data.data.begin(), modem_data.data.end());
		//Buffer one message like the acoustic modem does
		std_msgs::String reply;
		reply.data.assign(modem_data.data.begin(), modem_data.data.end());
		reply_queue.push(reply);
		if (reply_queue.size() > 1) reply_queue.pop();
		ROS_INFO("Received data message.");
	}

	void onDiverSim(const auv_msgs::NavSts::ConstPtr msg)
	{
		boost::mutex::scoped_lock l(vehicleStateMux);
		vehicleState = *msg;
	}

	void onPlatformSim(const auv_msgs::NavSts::ConstPtr msg)
	{
		if ((ros::Time::now() - lastNav).toSec() > navTime)
		{
			boost::mutex::scoped_lock l(vehicleStateMux);
			geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped());
			point->header.stamp = ros::Time::now();
			point->point.x = msg->position.north - vehicleState.position.north;
			point->point.y = msg->position.east - vehicleState.position.east;
			point->point.z = msg->position.depth - vehicleState.position.depth;
			usblNav.publish(point);
		}
	}

	void onOutgoingMsg(const std_msgs::String::ConstPtr msg)
	{
		using namespace labust::tritech;
		MTMsgPtr tmsg(new MTMsg());
		tmsg->txNode = 85;
		tmsg->rxNode = labust::tritech::Nodes::Surface;
		tmsg->node = 85;
		tmsg->msgType = MTMsg::mtMiniModemData;

		MMCMsg mmsg;
		mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits48;
		for (int i=0;i<msg->data.size();++i) mmsg.data[i] = msg->data[i];

		boost::archive::binary_oarchive ar(*tmsg->data, boost::archive::no_header);
		ar<<mmsg;
		usblBusy = true;

		if (useDevice)
		{
			usbl->send(tmsg);
		}
		else
		{
			onReplyMsg(tmsg);
		}
		ROS_INFO("Sent data message.");

		//boost::mutex::scoped_lock lock(pingLock);
		//while (usblBusy) usblCondition.wait(lock);
		if (!reply_queue.empty())
		{
			dataPub.publish(reply_queue.front());
			reply_queue.pop();
			sent = true;
		}
	}

	ros::Publisher dataPub, usblTimeout, usblNav;
	ros::Subscriber dataSub, simDiverState, simVehicleState;
	auv_msgs::NavSts vehicleState;
	boost::shared_ptr<labust::tritech::MTDevice> usbl;
	boost::mutex pingLock, vehicleStateMux;
	boost::condition_variable usblCondition;
	bool usblBusy, useDevice;
	std::queue<std_msgs::String> reply_queue;
	bool sent;
	double navTime;
	ros::Time lastNav;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"usbl_sim");
	USBLSim usbl;
	ros::spin();

	return 0;
}



