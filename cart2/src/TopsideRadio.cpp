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
 *  Created: 06.05.2013.
 *********************************************************************/
#include <labust/control/TopsideRadio.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <cart2/SetHLMode.h>
#include <cart2/HLMessage.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/bind.hpp>
#include <boost/regex.hpp>

#include <std_msgs/Bool.h>

#include <stdexcept>

using labust::control::TopsideRadio;

TopsideRadio::TopsideRadio():
				port(io),
				ringBuffer('0',7),
				isTopside(true){this->onInit();}

TopsideRadio::~TopsideRadio()
{
	io.stop();
	iorunner.join();
	if (client) client.shutdown();
}

void TopsideRadio::onInit()
{
	ros::NodeHandle nh,ph("~");

	std::string portName("/dev/ttyUSB0");
	int baud(9600);
	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);
	ph.param("IsTopside",isTopside,isTopside);

	port.open(portName);
	port.set_option(boost::asio::serial_port::baud_rate(baud));
	port.set_option(boost::asio::serial_port::flow_control(
			boost::asio::serial_port::flow_control::hardware));

	if (!port.is_open())
	{
		ROS_ERROR("Cannot open port.");
		throw std::runtime_error("Unable to open the port.");
	}

	if (isTopside)
	{
		extPoint = nh.subscribe<geometry_msgs::PointStamped>("target_point", 1,
				&TopsideRadio::onExtPoint,this);
		joyIn = nh.subscribe<sensor_msgs::Joy>("joy_in",1,&TopsideRadio::onJoy,this);
		//Dynamic reconfigure
		server.setCallback(boost::bind(&TopsideRadio::dynrec_cb, this, _1, _2));
		populateDataFromConfig();

		nh.param("LocalOriginLat",originLat,originLat);
		nh.param("LocalOriginLon",originLon,originLon);
	}
	else
	{
		joyOut = nh.advertise<sensor_msgs::Joy>("joy_out",1);
		launched = nh.advertise<std_msgs::Bool>("launched",1);
		hlMsg = nh.advertise<cart2::HLMessage>("hl_message",1);
		client = nh.serviceClient<cart2::SetHLMode>("SetHLMode", true);
	}

	populateDataFromConfig();

	this->start_receive();
	iorunner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
}

void TopsideRadio::onJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
	boost::mutex::scoped_lock l(dataMux);
	data.surgeForce = joy->axes[1];
	data.torqueForce = joy->axes[2];
}

void TopsideRadio::onExtPoint(const geometry_msgs::PointStamped::ConstPtr& extPoint)
{
	if (!config.ManualPoint)
	{
		boost::mutex::scoped_lock l(dataMux);
		if (extPoint->header.frame_id == "worldLatLon")
		{
			data.lat = extPoint->point.x;
			data.lon = extPoint->point.y;
		}
		else if (extPoint->header.frame_id == "local")
		{
			local2LatLon(extPoint->point.x, extPoint->point.y);
		}
		else
		{
			ROS_ERROR("The specified point for the radio modem has to be in worldLatLon frame!.");
		}
	}
}

void TopsideRadio::local2LatLon(double x, double y)
{
	std::pair<double, double> location = labust::tools::meter2deg(x,
			y,
			originLat);
	data.lat = originLat + location.first;
	data.lon = originLon + location.second;
}

void TopsideRadio::populateDataFromConfig()
{
	boost::mutex::scoped_lock l(dataMux);
	data.mode = config.OpMode;
	data.launch = config.Launch;
	if (config.ManualPoint)
	{
		if (config.UseLocal)
		{
			local2LatLon(config.PointN, config.PointE);
		}
		else
		{
			data.lat = config.PointLat;
			data.lon = config.PointLon;
		}
	}
	data.radius = config.Radius;
	data.surge = config.Surge;
}

void TopsideRadio::dynrec_cb(cart2::RadioModemConfig& config, uint32_t level)
{
	this->config = config;
	this->populateDataFromConfig();
}

void TopsideRadio::start_receive()
{
	boost::asio::async_read(port, sbuffer.prepare(sync_length),
			boost::bind(&TopsideRadio::onSync,this,_1,_2));
}

void TopsideRadio::onSync(const boost::system::error_code& error, const size_t& transferred)
{
	if (!error)
	{
		sbuffer.commit(transferred);
		if (ringBuffer.size()>sync_length) ringBuffer.erase(ringBuffer.begin());

		if (transferred == 1)
		{
			ringBuffer.push_back(sbuffer.sbumpc());
		}
		else
		{
			std::istream is(&sbuffer);
			is >> ringBuffer;
		}

		if (ringBuffer == "@CART2")
		{
			boost::asio::async_read(port,sbuffer.prepare(37),boost::bind(&TopsideRadio::onIncomingData,this,_1,_2));
		}
		else
		{
			boost::asio::async_read(port, sbuffer.prepare(1),
					boost::bind(&TopsideRadio::onSync,this,_1,_2));
		}
	}
}

void TopsideRadio::onIncomingData(const boost::system::error_code& error, const size_t& transferred)
{
	sbuffer.commit(transferred);
	boost::archive::binary_iarchive dataSer(sbuffer, boost::archive::no_header);

	if (!isTopside)
	{
			dataSer >> data;

			sensor_msgs::Joy joy;
			joy.axes.assign(6,0);
			joy.axes[1] = data.surgeForce;
			joy.axes[2] = data.torqueForce;
			joyOut.publish(joy);

			std_msgs::Bool launcher;
			launcher.data = data.launch;
			launched.publish(launcher);

			cart2::HLMessagePtr msg(new cart2::HLMessage());
			msg->mode = data.mode;
			msg->radius = data.radius;
			msg->ref_point.header.stamp=ros::Time::now();
			msg->ref_point.header.frame_id="worldLatLon";
			msg->ref_point.point.x = data.lat;
			msg->ref_point.point.y = data.lon;
			msg->surge = data.surge;
			hlMsg.publish(msg);

			if (!client)
			{
				ROS_ERROR("HLManager client not connected. Trying reset.");
				client = nh.serviceClient<cart2::SetHLMode>("SetHLMode", true);
			}
			else
			{
				cart2::SetHLMode mode;
				mode.request.mode = data.mode;
				mode.request.ref_point.header.stamp=ros::Time::now();
				mode.request.ref_point.header.frame_id="worldLatLon";
				mode.request.ref_point.point.x = data.lat;
				mode.request.ref_point.point.y = data.lon;
				mode.request.radius = data.radius;
				mode.request.surge = data.surge;
				client.call(mode);
			}
	}

	start_receive();
}

void TopsideRadio::start()
{
	if (!isTopside)
	{
		ros::spin();
	}
	else
	{
		ros::Rate rate(10);
		while (nh.ok())
		{
			boost::asio::streambuf output;
			std::ostream out(&output);
			//Prepare sync header
			out<<"@CART2";
			boost::archive::binary_oarchive dataSer(output, boost::archive::no_header);
			boost::mutex::scoped_lock l(dataMux);
			dataSer << data;
			l.unlock();

			ROS_INFO("Sending data.");

			//write data
			boost::asio::write(port, output.data());
			rate.sleep();
			ros::spinOnce();
		}
	}
}

