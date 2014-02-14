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
#include <labust/tritech/USBLNodelet.hpp>
#include <labust/tritech/TCPDevice.hpp>
#include <labust/tritech/mtMessages.hpp>
#include <labust/tritech/USBLMessages.hpp>
#include <labust/tools/conversions.hpp>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

PLUGINLIB_DECLARE_CLASS(usbl,USBLNodelet,labust::tritech::USBLNodelet, nodelet::Nodelet)

using namespace labust::tritech;

USBLNodelet::USBLNodelet():
	address("127.0.0.1"),
	port(4000),
	usblBusy(false),
	autoMode(true),
	ping_timeout(7){};

USBLNodelet::~USBLNodelet()
{
	stop();
	usbl.reset();
};

void USBLNodelet::onInit()
{
	ros::NodeHandle ph = this->getPrivateNodeHandle();
	ph.param("ip",address,address);
	ph.param("port",port,port);
	std::string op_mode("auto");
	ph.param("op_mode",op_mode,op_mode);
	autoMode = (op_mode == "auto");
	//Connect to the TCPDevice
	usbl.reset(new TCPDevice(address,port));
	attitude.reset(new TCPDevice(address,port, 
		labust::tritech::Nodes::AttitudeSensor,
		labust::tritech::TCPRequest::atMiniAttSen,
		127));
	//Register handlers
	TCPDevice::HandlerMap map;
	map[MTMsg::mtAlive] = boost::bind(&USBLNodelet::onTCONMsg,this,_1);
	map[MTMsg::mtAMNavDataV2] = boost::bind(&USBLNodelet::onNavMsg,this, _1);
	map[MTMsg::mtMiniAttData] = boost::bind(&USBLNodelet::onAttMsg, this, _1);
	//map[MTMsg::mtAMNavDataV2] = map[MTMsg::mtAMNavData];
	usbl->registerHandlers(map);
	attitude->registerHandlers(map);

	ros::NodeHandle nh = this->getNodeHandle();
	dataSub = nh.subscribe<std_msgs::String>("outgoing_data",	0, boost::bind(&USBLNodelet::onOutgoingMsg,this,_1));
	opMode = nh.subscribe<std_msgs::Bool>("auto_mode",	0, boost::bind(&USBLNodelet::onAutoMode,this,_1));
	navPub = nh.advertise<geometry_msgs::PointStamped>("usbl_nav",1);
	dataPub = nh.advertise<std_msgs::String>("incoming_data",1);
	usblTimeout = nh.advertise<std_msgs::Bool>("usbl_timeout",1);
	attRaw = nh.advertise<std_msgs::Float32MultiArray>("usbl_att_raw",1);
	attData = nh.advertise<geometry_msgs::PointStamped>("usbl_att",1);

	if (autoMode) worker = boost::thread(boost::bind(&USBLNodelet::run,this));
}

void USBLNodelet::stop()
{
	{
		boost::mutex::scoped_lock lock(pingLock);
		this->usblBusy = false;
		this->autoMode = false;
	}
	usblCondition.notify_all();
	worker.join();
}

void USBLNodelet::onAutoMode(const std_msgs::Bool::ConstPtr mode)
{
	//Turn off autoMode
	if (this->autoMode && !mode->data)
	{
		stop();
		NODELET_INFO("Turning off USBL auto interrogation.");
	}

	//Turn on autoMode
	if (!this->autoMode && mode->data)
	{
		this->autoMode = true;
		worker = boost::thread(boost::bind(&USBLNodelet::run,this));
		NODELET_INFO("Turning on USBL auto interrogation.");
	}
};

void USBLNodelet::onOutgoingMsg(const std_msgs::String::ConstPtr msg)
{
	boost::mutex::scoped_lock lock(dataMux);
	msg_out = msg->data;
	if (!autoMode) sendUSBLPkg();
};

void USBLNodelet::onTCONMsg(labust::tritech::TCONMsgPtr tmsg)
{
	if (tmsg->msgType == MTMsg::mtAlive)
	{
		NODELET_DEBUG("Recevied alive message.");
	}
	else
	{
		NODELET_DEBUG("Recevied TCONMsg %d.",tmsg->msgType);
	}
}

void USBLNodelet::onAttMsg(labust::tritech::TCONMsgPtr tmsg)
{
	boost::archive::binary_iarchive dataSer(*tmsg->data, boost::archive::no_header);
	AttSenData att_data;
	dataSer>>att_data;

	geometry_msgs::PointStamped::Ptr attOut(new geometry_msgs::PointStamped());
	std_msgs::Float32MultiArray::Ptr rawOut(new std_msgs::Float32MultiArray());

	rawOut->data.push_back(att_data.cmd);
	rawOut->data.push_back(att_data.time);
	rawOut->data.push_back(att_data.pressure);
	rawOut->data.push_back(att_data.internalTemp);
	rawOut->data.push_back(att_data.externalTemp);
	for (int i=0; i<3; ++i) rawOut->data.push_back(att_data.acc[i]);
	for (int i=0; i<3; ++i) rawOut->data.push_back(att_data.mag[i]);
	for (int i=0; i<3; ++i) rawOut->data.push_back(att_data.gyro[i]);

	attRaw.publish(rawOut);
}

void USBLNodelet::onNavMsg(labust::tritech::TCONMsgPtr tmsg)
{
	{
		boost::mutex::scoped_lock lock(pingLock);
		usblBusy = false;
	}
	usblCondition.notify_one();

	boost::archive::binary_iarchive dataSer(*tmsg->data, boost::archive::no_header);
	USBLData usbl_data;
	MMCMsg modem_data;

	if (tmsg->msgType == MTMsg::mtAMNavData)
	{
		//Decoding V1 message
		NODELET_DEBUG("Decoding V1 message type: %d\n",tmsg->msgType);
		dataSer>>usbl_data;
	}
	else
	{
		//Decoding V2 message
		NODELET_DEBUG("Decoding V2 message type: %d\n",tmsg->msgType);
		dataSer>>usbl_data>>modem_data;
	}

	geometry_msgs::PointStamped::Ptr usblOut(new geometry_msgs::PointStamped());

	if (usbl_data.reply_validity == 30)
	{
		usblOut->header.stamp = ros::Time::now();
		usblOut->header.frame_id = "usbl";

		usblOut->point.x = usbl_data.attitudeCorrectedPos[1];
		usblOut->point.y = usbl_data.attitudeCorrectedPos[0];
		usblOut->point.z = usbl_data.attitudeCorrectedPos[2];

		navPub.publish(usblOut);

		if (modem_data.msgType != mmcRangeData)
		{
			std_msgs::String::Ptr modem(new std_msgs::String());
			size_t size = modem_data.data[MMCMsg::ranged_payload_size]/8;
			std::cout<<"Modem data byte size "<<size<<" data:";
			for(int i=0; i<modem_data.data.size(); ++i) std::cout<<int(modem_data.data[i])<<",";
			std::cout<<std::endl;
			modem->data.assign(modem_data.data.begin() + MMCMsg::ranged_payload_size,
					modem_data.data.begin() + MMCMsg::ranged_payload_size + size + 1);
			dataPub.publish(modem);
		}
	}
	else
	{
		NODELET_DEBUG("Invalid data reply from USBL. Validity:%d\n",usbl_data.reply_validity);
		std_msgs::Bool data;
		data.data = true;
		usblTimeout.publish(data);
	}

	NODELET_DEBUG("Received data message.\n");
}

void USBLNodelet::sendUSBLPkg()
{
	TCONMsgPtr tmsg(new TCONMsg());
	tmsg->txNode = 255;
	tmsg->rxNode = labust::tritech::Nodes::USBL;
	tmsg->node = 255;
	tmsg->msgType = MTMsg::mtMiniModemCmd;

	MMCMsg mmsg;
	mmsg.msgType = labust::tritech::mmcGetRangeSync;

	if (msg_out.size())
	{
		//Switch message size
		if (msg_out[0] == 48) mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits48;
		//mmsg.data[0] = 48;
		//for (int i=0;i<msg_out.size();++i) mmsg.data[i+1] = msg_out[i];
		for (int i=0;i<msg_out.size();++i) mmsg.data[i] = msg_out[i];
		msg_out.clear();
	}

	boost::archive::binary_oarchive ar(*tmsg->data, boost::archive::no_header);
	ar<<mmsg;
	usbl->send(tmsg);
	usblBusy = true;

	boost::mutex::scoped_lock lock(pingLock);
	boost::system_time const timeout=boost::get_system_time()+boost::posix_time::seconds(ping_timeout);
	while (usblBusy) 
	{
		if (!usblCondition.timed_wait(lock,timeout))
		{ 
			NODELET_INFO("USBL went into timeout.");
			std_msgs::Bool data;
			data.data = true;
			usblTimeout.publish(data);
			break;
		}
	}
}

void USBLNodelet::run()
{
	while (ros::ok() && autoMode)
	{
		sendUSBLPkg();
		//Broadcast frame
		///\todo Determine if transform broadcast is neeeded
		///\todo add transform settings in initial parameters.
		///\todo determine real transform values relative to base_link
		geometry_msgs::TransformStamped transform;
		transform.transform.translation.x = 0.25;
		transform.transform.translation.y = 0;
		transform.transform.translation.z = 0.5;
		labust::tools::quaternionFromEulerZYX(0, 0, 0,
				transform.transform.rotation);
		transform.child_frame_id = "usbl_frame";
		transform.header.frame_id = "base_link";
		transform.header.stamp = ros::Time::now();
		frameBroadcast.sendTransform(transform);
		NODELET_INFO("Running.");	
	}
	NODELET_INFO("Exiting run.");	
}


