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
#include <boost/thread.hpp>

#include <string>

struct SharedData
{
	SharedData():socket(io){};
	boost::asio::io_service io;
	boost::asio::ip::udp::socket socket;
	boost::asio::ip::udp::endpoint remote;
	ros::Publisher trackPoint;
	uint8_t buffer[10000];
};

void onData(SharedData& shared, const auv_msgs::NavSts::ConstPtr data)
{
	std::pair<double, double> location = labust::tools::meter2deg(data->position.north, data->position.east,
			data->origin.latitude);
	std::vector<int32_t> buffer(2);
	int32_t latdeg = int32_t((data->origin.latitude + location.first)*10000);
	int32_t londeg = int32_t((data->origin.longitude + location.second)*10000);
	buffer[0] = htonl(latdeg);
	//buffer[1] = htonl(int32_t((data->origin.latitude-latdeg)*10000));
	buffer[1] = htonl(londeg);
	//buffer[3] = htonl(int32_t((data->origin.longitude-londeg)*10000));

	uint8_t* p=reinterpret_cast<uint8_t*>(buffer.data());
	uint32_t len =  buffer.size()*sizeof(int32_t);
	//	uint8_t out[len];
	//	for(int32_t i=0; i<buffer.size();++i)
	//	{
	//		out[i] = p[len-i-1];
	//	}
	shared.socket.send_to(boost::asio::buffer(p, len), shared.remote);
	//std::string test = "Test\n";
	//shared.socket.send_to(boost::asio::buffer(test), shared.remote);
	std::cout<<"Send data lat:"<<latdeg<<std::endl;
	std::cout<<"Send data lon:"<<londeg<<std::endl;
}

void handle_receive(SharedData& shared, const boost::system::error_code& e, size_t length);
void start_receive(SharedData& shared)
{
	shared.socket.async_receive(boost::asio::buffer(shared.buffer, sizeof(shared.buffer)),
			boost::bind(&handle_receive,boost::ref(shared),_1,_2));
}

void handle_receive(SharedData& shared, const boost::system::error_code& e, size_t length)
{
	if (!e)
	{
		int32_t* pos = reinterpret_cast<int32_t*>(&shared.buffer[0]);

		geometry_msgs::PointStamped point;
		point.header.frame_id = "worldLatLon";
		point.point.x = htonl(pos[0])/10000.;
		point.point.y = htonl(pos[1])/10000.;
		point.point.z = 0;

		shared.trackPoint.publish(point);
	}
	else
	{
		ROS_ERROR("Error in communication.");
	}

	start_receive(shared);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"coop_node");
	ros::NodeHandle nh,ph("~");

	SharedData shared;
	ros::Subscriber measIn = nh.subscribe<auv_msgs::NavSts>("meas",1,boost::bind(&onData,boost::ref(shared),_1));
	shared.trackPoint = nh.advertise<geometry_msgs::PointStamped>("extPoint",1);

	using namespace boost::asio::ip;
	//Get thruster configuration
	std::string ip("127.0.0.1"),port("8692"), port_local("33346");
	ph.param("ip", ip,ip);
	ph.param("port", port,port);
	ph.param("port_local", port_local,port_local);
	shared.socket.open(udp::v4());
	//Create a name resolver
	udp::resolver resolver(shared.io);
	//Resolve query and get the iterator
	shared.remote = *resolver.resolve(udp::resolver::query(ip,port));
	udp::resolver::iterator it = resolver.resolve(udp::resolver::query(
			"150.145.4.103",port_local));
	boost::system::error_code error = boost::asio::error::host_not_found;
	while (error && (it != udp::resolver::iterator()))
	{
		shared.socket.bind(*it++,error);
	}

	start_receive(shared);
	boost::thread t(boost::bind(&boost::asio::io_service::run, &shared.io));

	ros::spin();
	shared.io.stop();
	//t.join();
	return 0;
}




