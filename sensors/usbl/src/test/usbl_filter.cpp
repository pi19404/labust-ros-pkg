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
*  Created: 18.02.2013.
*********************************************************************/
#include <underwater_sensor_msgs/USBL.h>

#include <ros/ros.h>


void onMsg(labust::tritech::TCPDevice* usbl, const std_msgs::String::ConstPtr msg){};

void onNavMsg(ros::Publisher* usblNav, ros::Publisher* usblMsg, labust::tritech::TCONMsgPtr tmsg){}

int main(int argc, char* argv[])
{
	using namespace labust::tritech;

	ros::init(argc,argv,"usbl_filter");

	ros::Subscriber usblData = nh.subscribe<underwater_sensor_msgs::USBL>(
			"modem_data",	1,boost::bind(&onMsg,&usbl,_1));
	ros::Publisher pub = nh.advertise<underwater_sensor_msgs::USBL>("usbl_nav",1);
	ros::Publisher pub2 = nh.advertise<std_msgs::String>("usbl_data",1);

	TCPDevice::HandlerMap map;
	map[MTMsg::mtAMNavDataV2] = boost::bind(&onNavMsg,&pub,&pub2, _1);
	usbl.registerHandlers(map);

	ros::spin();

	return 0;
}



