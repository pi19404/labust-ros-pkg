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
 *  Created: 23.01.2013.
 *********************************************************************/
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <boost/regex.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <iostream>

void handleJoy(boost::asio::serial_port& port,const sensor_msgs::Joy::ConstPtr& joy)
{
	std::ostringstream out;

	out.precision(8);
	out<<"JOY";
	for (size_t i=0; i<4;++i)
	{
		out<<" "<<joy->axes[i];
	}
	out<<"\r"<<std::endl;

	boost::asio::write(port, boost::asio::buffer(out.str()));
};

void handleOutgoing(boost::asio::serial_port& port, const std_msgs::String::ConstPtr& msg)
{
	if ((msg->data[msg->data.length()-1]) != '\n')
	{
		std::string message(msg->data);
		message.append("\r\n");
		boost::asio::write(port, boost::asio::buffer(message));
	}
	else boost::asio::write(port, boost::asio::buffer(msg->data));
}

void start_receive(ros::Publisher& pub,
		ros::Publisher& joy,
		boost::asio::streambuf& sbuffer,
		boost::asio::serial_port& port);

void handleIncoming(ros::Publisher& pub,
		ros::Publisher& joyPub,
		boost::asio::streambuf& sbuffer,
		boost::asio::serial_port& port,
		const boost::system::error_code& error, const size_t& transferred)
{
	std_msgs::String data;
	std::istream is(&sbuffer);
	std::getline(is, data.data);

	if (data.data.substr(0,3) == "JOY")
	{
		data.data.erase(0,4);

		sensor_msgs::Joy joy;
		std::istringstream in(data.data);
		for (int i=0; i<4; ++i)
		{
			float temp;
			in>>temp;
			joy.axes.push_back(temp);
		}
		joyPub.publish(joy);
	}
	else pub.publish(data);

	start_receive(pub,joyPub,sbuffer,port);
}

void start_receive(ros::Publisher& pub,
		ros::Publisher& joy,
		boost::asio::streambuf& sbuffer,
		boost::asio::serial_port& port)
{
	boost::asio::async_read_until(port, sbuffer, boost::regex("\n"),
				boost::bind(&handleIncoming,
						boost::ref(pub),
						boost::ref(joy),
						boost::ref(sbuffer),
						boost::ref(port),_1,_2));
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"vr_node");
	ros::NodeHandle nh,ph("~");

	std::string portName("/dev/ttyUSB0");
	int baud(9600);

	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);

	namespace ba=boost::asio;
	ba::io_service io;
	ba::serial_port port(io);

	port.open(portName);
	port.set_option(ba::serial_port::baud_rate(baud));
	port.set_option(ba::serial_port::flow_control(
			ba::serial_port::flow_control::hardware));

	if (!port.is_open())
	{
		std::cerr<<"Cannot open port."<<std::endl;
		exit(-1);
	}

	ros::Publisher dataIn = nh.advertise<std_msgs::String>("incoming_data",1);
	ros::Publisher joyPub = nh.advertise<sensor_msgs::Joy>("joy_out",1);
	ros::Subscriber dataOut = nh.subscribe<std_msgs::String>("outgoing_data", 1, boost::bind(&handleOutgoing,boost::ref(port),_1));
	ros::Subscriber joy = nh.subscribe<sensor_msgs::Joy>("joy_in",1,boost::bind(&handleJoy,boost::ref(port),_1));

	boost::asio::streambuf sbuffer;

	start_receive(dataIn,joyPub,sbuffer,port);
	boost::thread t(boost::bind(&ba::io_service::run,&io));

	ros::spin();
	io.stop();
	t.join();

	return 0;
}

