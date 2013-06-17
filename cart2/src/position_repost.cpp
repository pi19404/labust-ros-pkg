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
#include <labust/tools/StringUtilities.hpp>
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

uint8_t aisConvertByte(uint8_t byte)
{
  if (byte > 40)
  {
  	byte += 56;
  }
  else
  {
    byte += 48;
  }

  return byte;
}

void aivdm2(long lat, long lon, int hdg, std::string& ais, bool isTarget = false)
{
	if (isTarget)
	{
		 ais = "!AIVDM,1,1,,A,13u?etP00000000000000000069D,0*  \n";
	}
	else
	{
		ais = "!AIVDM,1,1,,A,13u?ftP00000000000000000069D,0*  \n";
	}
  ROS_INFO("Encoding AIS information: lat=%d, lon=%d hdg=%d",lat,lon,hdg);
	unsigned char chk;
	int i;
	ais[24] = 0x0 | ((lon >> 23) & 0x1F);
	ais[25] = ((lon >> 17) & 0x3F);
	ais[26] = ((lon >> 11) & 0x3F);
	ais[27] = ((lon >> 5) & 0x3F);
	ais[28] = ((lon & 0x1F) << 1) | ((lat >> 26) & 0x01);
	ais[29] = ((lat >> 20) & 0x3F);
	ais[30] = ((lat >> 14) & 0x3F);
	ais[31] = ((lat >> 8) & 0x3F);
	ais[32] = ((lat >> 2) & 0x3F);
	ais[33] = ((lat & 0x3) << 4) | 0x0;
	ais[34] = 0;
	ais[35] = ((hdg >> 5) & 0xF);
	ais[36] = ((hdg & 0x1F) << 1);
	ais[37] = 0;
	for(int i=24; i<=37; ++i) ais[i] = aisConvertByte(uint8_t(ais[i]));
  chk = labust::tools::getChecksum(reinterpret_cast<const uint8_t*>(ais.data()), 44);
  if((chk >> 4) < 10)
  {
      ais[45] = (chk >> 4) + 48;
  }
  else
  {
      ais[45] = (chk >> 4) + 55;
  }

  if((chk & 0xF) < 10)
  {
      ais[46] = (chk & 0xF) + 48;
  }
  else
  {
      ais[46] = (chk & 0xF) + 55;
  }
//  for (i=0; i<47; i++) {
//      printf("%c", ais[i]);
//  }
//  printf("\n");
}

void aivdm(long lat, long lon, int hdg) {
    unsigned char ais[47] = {'!', 'A', 'I', 'V', 'D', 'M', ',', '1', ',', '1', ',', ',', 'A', ',', '1', '3', 'u', '?', 'e', 't', 'P', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '6', '9', 'D', ',', '0', '*', ' ', ' '};
    unsigned char chk;
    int i;
    ais[24] = 0x0 | ((lon >> 23) & 0x1F);
    ais[25] = ((lon >> 17) & 0x3F);
    ais[26] = ((lon >> 11) & 0x3F);
    ais[27] = ((lon >> 5) & 0x3F);
    ais[28] = ((lon & 0x1F) << 1) | ((lat >> 26) & 0x01);
    ais[29] = ((lat >> 20) & 0x3F);
    ais[30] = ((lat >> 14) & 0x3F);
    ais[31] = ((lat >> 8) & 0x3F);
    ais[32] = ((lat >> 2) & 0x3F);
    ais[33] = ((lat & 0x3) << 4) | 0x0;
    ais[34] = 0;
    ais[35] = ((hdg >> 5) & 0xF);
    ais[36] = ((hdg & 0x1F) << 1);
    ais[37] = 0;
    if (ais[24] > 40)
        ais[24] += 56;
    else
        ais[24] += 48;
    if (ais[25] > 40)
        ais[25] += 56;
    else
        ais[25] += 48;
    if (ais[26] > 40)
        ais[26] += 56;
    else
        ais[26] += 48;
    if (ais[27] > 40)
        ais[27] += 56;
    else
        ais[27] += 48;
    if (ais[28] > 40)
        ais[28] += 56;
    else
        ais[28] += 48;
    if (ais[29] > 40)
        ais[29] += 56;
    else
        ais[29] += 48;
    if (ais[30] > 40)
        ais[30] += 56;
    else
        ais[30] += 48;
    if (ais[31] > 40)
        ais[31] += 56;
    else
        ais[31] += 48;
    if (ais[32] > 40)
        ais[32] += 56;
    else
        ais[32] += 48;
    if (ais[33] > 40)
        ais[33] += 56;
    else
        ais[33] += 48;
    if (ais[34] > 40)
        ais[34] += 56;
    else
        ais[34] += 48;
    if (ais[35] > 40)
        ais[35] += 56;
    else
        ais[35] += 48;
    if (ais[36] > 40)
        ais[36] += 56;
    else
        ais[36] += 48;
    if (ais[37] > 40)
        ais[37] += 56;
    else
        ais[37] += 48;
    chk = labust::tools::getChecksum(ais, 44);
    if((chk >> 4) < 10)
        ais[45] = (chk >> 4) + 48;
    else
        ais[45] = (chk >> 4) + 55;
    if((chk & 0xF) < 10)
        ais[46] = (chk & 0xF) + 48;
    else
        ais[46] = (chk & 0xF) + 55;
    for (i=0; i<47; i++) {
        printf("%c", ais[i]);
    }
    printf("\n");
}

void onData(SharedData& shared, bool isTarget, const auv_msgs::NavSts::ConstPtr data)
{
	std::string ais_data;
	int hdg = int(data->orientation.yaw*180/M_PI);
	if (hdg < 0) hdg +=360;
	aivdm2(long(data->global_position.latitude*600000),
			long(data->global_position.longitude*600000),
			hdg, ais_data, isTarget);

	ROS_INFO("Encoded ais: %s",ais_data.c_str());

	boost::asio::write(shared.port,boost::asio::buffer(ais_data));

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
	ros::Subscriber cartPos = nh.subscribe<auv_msgs::NavSts>("cartPos",1,boost::bind(&onData,boost::ref(shared),false,_1));
	ros::Subscriber bartPos = nh.subscribe<auv_msgs::NavSts>("bartPos",1,boost::bind(&onData,boost::ref(shared),true,_1));
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
