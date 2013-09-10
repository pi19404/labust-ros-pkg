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

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

///\todo Join the USBLSim and real USBL classes ??
class USBLSim
{
public:
	USBLSim():
		usblBusy(false),
		useDevice(false)
	{
		ros::NodeHandle nh,ph("~");
		std::string port("/dev/rfcomm0");
		int baud(57600);

		ph.param("port",port,port);
		ph.param("baud",baud,baud);
		ph.param("use_device",useDevice,useDevice);

		if (useDevice)
		{
			using namespace labust::tritech;
			usbl.reset(new MTDevice(port,baud));
			MTDevice::HandlerMap map;
			map[MTMsg::mtMiniModemCmd] = boost::bind(&USBLSim::onReplyMsg,this, _1);
			usbl->registerHandlers(map);
		}

		dataSub = nh.subscribe<std_msgs::String>("outgoing_data",	0, boost::bind(&USBLSim::onOutgoingMsg,this,_1));
		dataPub = nh.advertise<std_msgs::String>("incoming_data",1);
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
		//std_msgs::String::Ptr modem(new std_msgs::String());
		//modem->data.assign(modem_data.data.begin(), modem_data.data.end());
		dataPub.publish(last_reply);
		//Buffer one message like the acoustic modem does
		last_reply.data.assign(modem_data.data.begin(), modem_data.data.end());
		ROS_INFO("Received data message.");
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
	}

	ros::Subscriber dataSub;
	ros::Publisher dataPub;
	boost::shared_ptr<labust::tritech::MTDevice> usbl;
	boost::mutex pingLock;
	boost::condition_variable usblCondition;
	bool usblBusy, useDevice;
	std_msgs::String last_reply;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"usbl_sim");
	USBLSim usbl;
	ros::spin();

	return 0;
}



