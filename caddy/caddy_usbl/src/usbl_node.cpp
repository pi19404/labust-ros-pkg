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
#include <labust/tritech/TCPDevice.hpp>
#include <labust/tritech/mtMessages.hpp>
#include <labust/tritech/USBLMessages.hpp>
#include <underwater_sensor_msgs/USBL.h>

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

boost::condition_variable usblCondition;
boost::mutex usblMux;
bool usblBusy(false);

void onMsg(labust::tritech::TCPDevice* usbl, const std_msgs::String::ConstPtr msg)
{
	boost::mutex::scoped_lock lock(usblMux);
	while (usblBusy) 	usblCondition.wait(lock);

	using namespace labust::tritech;

	TCONMsgPtr tmsg(new TCONMsg());
	tmsg->txNode = 255;
	tmsg->rxNode = labust::tritech::Nodes::USBL;
	tmsg->node = labust::tritech::Nodes::USBL;;
	tmsg->msgType = MTMsg::mtMiniModemCmd;

	std::istringstream s;
	//int id = msg->data.size();

	MMCMsg mmsg;
	mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits24;
	mmsg.data[0] = 24;
	for (int i=0;i<3;++i) 	mmsg.data[i+1] = msg->data[i];

	/*switch (msg->data.size())
	{
	 case 0: mmsg.msgType = labust::tritech::mmcGetRangeSync; break;
	 case 1: mmsg.msgType = labust::tritech::mmcGetRangeTxRxByte; break;
	 case 2: case 3: mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits24; break;
	 case 4: mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits32; break;
	 case 5: mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits40; break;
	 case 6: mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits48; break;
   default:break;
	}

	if (msg->data.size()) >
	mmsg.data[0] = msg->data.size();
	mmsg.data[1] = 50;
	mmsg.data[2] = 49;
	mmsg.data[3] = 48;
	mmsg.data[4] = 55;
	mmsg.data[5] = 56;
	mmsg.data[6] = 57;
*/
	std::ostream out();
	boost::archive::binary_oarchive ar(*tmsg->data, boost::archive::no_header);
	ar<<mmsg;

	usbl->send(tmsg);
	usblBusy = true;
};

void onNavMsg(ros::Publisher* usblNav, ros::Publisher* usblMsg, labust::tritech::TCONMsgPtr tmsg)
{
	using namespace labust::tritech;

	{
		boost::mutex::scoped_lock lock(usblMux);
		usblBusy = false;
	}
	usblCondition.notify_one();

	boost::archive::binary_iarchive dataSer(*tmsg->data, boost::archive::no_header);
	USBLDataV2 usbl_data;
	dataSer>>usbl_data;

	if (usbl_data.nav.reply_validity != 30)
	{
		std::cerr<<"Invalid reply form USBL."<<std::endl;
		//return;
	}

	underwater_sensor_msgs::USBL usblOut;

	usblOut.slant_range = usbl_data.nav.range;
	usblOut.position.x = usbl_data.nav.attitudeCorrectedPos[0];
	usblOut.position.y = usbl_data.nav.attitudeCorrectedPos[1];
	usblOut.position.z = usbl_data.nav.attitudeCorrectedPos[2];
	usblOut.bearing = atan2(usblOut.position.y, usblOut.position.x);
	usblOut.id = usbl_data.nav.unitID;
	usblOut.header.stamp = ros::Time::now();

	usblNav->publish(usblOut);

	std_msgs::String modem;
	modem.data.assign(usbl_data.modem.data.begin(), usbl_data.modem.data.end());
	usblMsg->publish(modem);

	std::cout<<"Received data message."<<std::endl;
}

int main(int argc, char* argv[])
{
	using namespace labust::tritech;

	ros::init(argc,argv,"usbl_node");

	ros::NodeHandle nh,ph("~");

	std::string address("127.0.0.1");
	int port(4000);

	ph.param("ip",address,address);
	ph.param("port",port,port);

	TCPDevice usbl(address,port);

	ros::Subscriber inMsg = nh.subscribe<std_msgs::String>(
			"modem_data",	1,boost::bind(&onMsg,&usbl,_1));
	ros::Publisher pub = nh.advertise<underwater_sensor_msgs::USBL>("usbl_nav",1);
	ros::Publisher pub2 = nh.advertise<std_msgs::String>("usbl_data",1);

	TCPDevice::HandlerMap map;
	map[MTMsg::mtAMNavDataV2] = boost::bind(&onNavMsg,&pub,&pub2, _1);
	usbl.registerHandlers(map);

	ros::spin();

	return 0;
}



