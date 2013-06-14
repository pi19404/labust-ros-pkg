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
 *  Created: 27.05.2013.
 *********************************************************************/
#include <boost/asio.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <ros/ros.h>
#include <auv_msgs/NavSts.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <string>

struct SharedData
{
	SharedData():port(io){};
	boost::asio::io_service io;
	boost::asio::serial_port port;
	ros::Publisher trackPoint;
	tf::TransformListener listener;
	uint8_t buffer[10000];
};

void onData(SharedData& shared, const auv_msgs::NavSts::ConstPtr data)
{
	try
	{
		tf::StampedTransform transformLocal;
		shared.listener.lookupTransform("worldLatLon", "local", ros::Time(0), transformLocal);
		std::pair<double, double> location = labust::tools::deg2meter(
				data->global_position.latitude - transformLocal.getOrigin().y(),
				data->global_position.longitude - transformLocal.getOrigin().x(),
				transformLocal.getOrigin().y());

		geometry_msgs::PointStamped point;
		point.header.frame_id = "local";
		point.point.x = location.first;
		point.point.y = location.second;
		point.point.z = 0;
		shared.trackPoint.publish(point);
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"repost_node");
	ros::NodeHandle nh,ph("~");

	SharedData shared;
	ros::Subscriber bartPos = nh.subscribe<auv_msgs::NavSts>("stateHat",1,boost::bind(&onData,boost::ref(shared),_1));
	shared.trackPoint = nh.advertise<geometry_msgs::PointStamped>("target_point",1);

	using namespace boost::asio::ip;
	//Get thruster configuration
	std::string portName("/dev/ttyS0");
	int baud(38400);
	ph.param("PortName", portName,portName);
	ph.param("BaudRate", baud,baud);
	shared.port.open(portName);
	shared.port.set_option(boost::asio::serial_port::baud_rate(baud));
	if (!shared.port.is_open())
	{
		ROS_ERROR("Cannot open port.");
		throw std::runtime_error("Unable to open the port.");
	}

	ros::spin();
	shared.io.stop();
	return 0;
}
