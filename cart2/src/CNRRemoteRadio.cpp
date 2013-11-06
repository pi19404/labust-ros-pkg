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
#include <labust/control/CNRRemoteRadio.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>
#include <navcon_msgs/SetHLMode.h>
#include <navcon_msgs/HLMessage.h>
#include <labust/control/crc16.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <boost/crc.hpp>

#include <std_msgs/Bool.h>

#include <stdexcept>
#include <bitset>

using labust::control::CNRRemoteRadio;

CNRRemoteRadio::CNRRemoteRadio():
						ph("~"),
						lastModemMsg(ros::Time::now()),
						timeout(10),
						currYaw(0),
						desiredHeading(0),
						buoyDistance(10),
						currLat(0),
						currLon(0),
						yawInc(0.5),
						port(io),
						buffer(sync_length,0),
						id(bart),
						doDummyRequest(false),
						doLaunch(false),
						wasLaunched(false),
						lastmode(1)
{this->onInit();}

CNRRemoteRadio::~CNRRemoteRadio()
{
	io.stop();
	iorunner.join();
	if (client) client.shutdown();
}

void CNRRemoteRadio::onInit()
{
	ros::NodeHandle nh,ph("~");

	std::string portName("/dev/ttyUSB0");
	int baud(9600);
	ph.param("PortName",portName,portName);
	ph.param("BaudRate",baud,baud);
	ph.param("Timeout",timeout,timeout);
	ph.param("ID",id,id);
	ph.param("YawInc",yawInc,yawInc);
	ph.param("DummyRequester",doDummyRequest,doDummyRequest);
	ph.param("BuoyDistance",doDummyRequest,doDummyRequest);

	port.open(portName);
	port.set_option(boost::asio::serial_port::baud_rate(baud));

	if (!port.is_open())
	{
		ROS_ERROR("Cannot open port.");
		throw std::runtime_error("Unable to open the port.");
	}

	joyOut = nh.advertise<sensor_msgs::Joy>("joy_out",1);
	posOut = nh.advertise<auv_msgs::NavSts>("bart_position",1);
	posCOut = nh.advertise<auv_msgs::NavSts>("cart_position",1);
	launched = nh.advertise<std_msgs::Bool>("launched",1);
	hlMsg = nh.advertise<navcon_msgs::HLMessage>("hl_message",1);
	client = nh.serviceClient<navcon_msgs::SetHLMode>("SetHLMode", true);
	stateHat = nh.subscribe<auv_msgs::NavSts>("stateHat",1,&CNRRemoteRadio::onStateHat,this);
	//stateMeas = nh.subscribe<auv_msgs::NavSts>("meas",1,&CNRRemoteRadio::onStateMeas,this);
	curMode = nh.subscribe<std_msgs::Int32>("current_mode",1,&CNRRemoteRadio::onCurrentMode,this);
	launchFlag = nh.subscribe<std_msgs::Bool>("set_launch",1,&CNRRemoteRadio::onLaunch,this);

	this->start_receive();
	iorunner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
}

void CNRRemoteRadio::onCurrentMode(const std_msgs::Int32::ConstPtr& mode)
{}

void CNRRemoteRadio::onLaunch(const std_msgs::Bool::ConstPtr& launch)
{
	doLaunch = launch->data;
}

void CNRRemoteRadio::onStateHat(const auv_msgs::NavSts::ConstPtr& estimate)
{
	boost::mutex::scoped_lock l(cdataMux);
	currYaw = estimate->orientation.yaw;
	currLat = estimate->global_position.latitude;
	currLon = estimate->global_position.longitude;
}

void CNRRemoteRadio::start_receive()
{
	boost::asio::async_read(port, boost::asio::buffer(&buffer[0],sync_length),
			boost::bind(&CNRRemoteRadio::onSync,this,_1,_2));
}

void CNRRemoteRadio::onSync(const boost::system::error_code& error, const size_t& transferred)
{
	if (!error)
	{
		if (transferred == 1)
		{
			//\todo revise this to be more general if needed
			ringBuffer[0] = ringBuffer[1];
			ringBuffer[1] = ringBuffer[2];
			ringBuffer[2] = buffer[0];
		}
		else
		{
			ringBuffer[0] = buffer[0];
			ringBuffer[1] = buffer[1];
			ringBuffer[2] = buffer[2];
		}

		if ((ringBuffer[0] == 'C') && (ringBuffer[1]=='P'))
		{
			ROS_INFO("Synced on CP.");
			buffer.resize(sync_length + ringBuffer[2] + chksum_size);
			buffer[0] = ringBuffer[0];
			buffer[1] = ringBuffer[1];
			buffer[2] = ringBuffer[2];
			ROS_INFO("Reading %d bytes.", ringBuffer[2] + chksum_size);
			boost::asio::async_read(port,boost::asio::buffer(&buffer[3], ringBuffer[2] + chksum_size),boost::bind(&CNRRemoteRadio::onIncomingData,this,_1,_2));
		}
		else
		{
			ROS_INFO("No sync: %d %d %d",ringBuffer[0],ringBuffer[1],ringBuffer[2]);
			boost::asio::async_read(port, boost::asio::buffer(&buffer[0],1),
					boost::bind(&CNRRemoteRadio::onSync,this,_1,_2));
		}
	}
}

void CNRRemoteRadio::onIncomingData(const boost::system::error_code& error, const size_t& transferred)
{
	int dataLen = sync_length + transferred - chksum_size;
	uint16_t chksum = compute_crc16(reinterpret_cast<const char*>(&buffer[0]),dataLen);
	uint16_t rcvchksum = 256*buffer[dataLen] + buffer[dataLen+1];

	if (chksum == rcvchksum)
	{
		int32_t recv = (buffer[id_field] & 0xF0) >> 4;
		int32_t sender = buffer[id_field] & 0x0F;

		ROS_INFO("Message from %d to %d.",sender,recv);


		if ((this->id == station) && (sender == bart))
		{
			int32_t data1=static_cast<int32_t>(
					htonl(*reinterpret_cast<uint32_t*>(&buffer[data1_field])));
			int32_t data2=static_cast<int32_t>(
					htonl(*reinterpret_cast<uint32_t*>(&buffer[data2_field])));

			lastModemMsg = ros::Time::now();
			auv_msgs::NavSts currPose;
			currPose.global_position.latitude = data1/double(latlonmux);
			currPose.global_position.longitude = data2/double(latlonmux);
			posOut.publish(currPose);
		}

		if ((this->id == station) && (sender == cart))
		{
			int32_t data1=static_cast<int32_t>(
					htonl(*reinterpret_cast<uint32_t*>(&buffer[data1_field+1])));
			int32_t data2=static_cast<int32_t>(
					htonl(*reinterpret_cast<uint32_t*>(&buffer[data2_field+1])));
			int16_t hdg=static_cast<int16_t>(
								htons(*reinterpret_cast<int16_t*>(&buffer[data2_field+5])));

			ROS_INFO("Received from CART: %d %d %d", data1,data2,hdg);

			lastModemMsg = ros::Time::now();
			auv_msgs::NavSts currPose;
			currPose.global_position.latitude = data1/double(latlonmux);
			currPose.global_position.longitude = data2/double(latlonmux);
			currPose.orientation.yaw = labust::math::wrapRad(hdg/100./180.*M_PI);
			posCOut.publish(currPose);
		}

		if ((this->id == bart) && (recv == bart))
		{
			if (buffer[launch_field] == 1)
			{
				ROS_INFO("The vehicle will be launched.");
				if (!wasLaunched)
				{
					std_msgs::Bool launcher;
					launcher.data = true;
					launched.publish(launcher);
					doLaunch = true;
				}

				if (wasLaunched)
				{
					//Stop the vehicle
					navcon_msgs::SetHLMode mode;
					while (!client)
					{
						ROS_ERROR("HLManager client not connected. Trying reset.");
						client = nh.serviceClient<navcon_msgs::SetHLMode>("SetHLMode", true);
					}

					ROS_INFO("Was launched - Request stop launch.");
					client.call(mode);
					doLaunch = false;
					wasLaunched = false;
				}
			}
			else
			{
				if (doLaunch)
				{
					wasLaunched = true;
				}

				if (!doLaunch)
				{
						ROS_INFO("Request stop launch.");
						//Stop the vehicle
						navcon_msgs::SetHLMode mode;
						while (!client)
						{
							ROS_ERROR("HLManager client not connected. Trying reset.");
							client = nh.serviceClient<navcon_msgs::SetHLMode>("SetHLMode", true);
						}

						client.call(mode);
				}

				if (wasLaunched)
				{
					ROS_INFO("Was launched.");
				}
			}
			boost::mutex::scoped_lock l2(clientMux);
			lastModemMsg = ros::Time::now();
			this->replyBuoy();
		}

		if ((this->id == cart) && (recv == cart))
		{
			lastmode = buffer[mode_field];
			std::bitset<8> modes(buffer[mode_field]);
			navcon_msgs::HLMessagePtr msg(new navcon_msgs::HLMessage());
			navcon_msgs::SetHLMode mode;

			if (modes[stopbit])
			{
				ROS_INFO("Switch to stop mode.");
				mode.request.mode = mode.request.Stop;
			}
			else
			{
				int32_t data1=static_cast<int32_t>(
						htonl(*reinterpret_cast<uint32_t*>(&buffer[data1_field])));
				int32_t data2=static_cast<int32_t>(
						htonl(*reinterpret_cast<uint32_t*>(&buffer[data2_field])));

				if (modes[manualbit])
				{
					mode.request.mode = mode.request.Manual;
					sensor_msgs::Joy joy;
					joy.axes.assign(6,0);
					joy.axes[1] = data2/10000.;
					joy.axes[2] = -data1/10000.;
					//Sanity check
					if (fabs(joy.axes[1]) > 1)
					{
						ROS_ERROR("Remote joystick force is above 1: %f", joy.axes[1]);
						joy.axes[1] = 0;
					}
					if (fabs(joy.axes[2]) > 1)
					{
						ROS_ERROR("Remote joystick force is above 1: %f",joy.axes[2]);
						joy.axes[2] = 0;
					}
					joyOut.publish(joy);

					ROS_INFO("Switch to manual mode: %f %f",
							joy.axes[1],
							joy.axes[2]);
				}

				if (modes[automaticbit])
				{
					mode.request.mode = mode.request.HeadingControl;
					sensor_msgs::Joy joy;
					joy.axes.assign(6,0);
					joy.axes[1] = data2/10000.;
					if (fabs(joy.axes[1]) > 1)
					{
						ROS_ERROR("Remote joystick force is above 1:", joy.axes[1]);
						joy.axes[1] = 0;
					}
					joyOut.publish(joy);

					boost::mutex::scoped_lock l(cdataMux);
					desiredHeading += data1/10000.*yawInc;
					mode.request.yaw = labust::math::wrapRad(desiredHeading);

					ROS_INFO("Automatic mode state: %d %d",
							data2,data1);

					ROS_INFO("Switch to automatic mode: %f %f",
							joy.axes[1],
							mode.request.yaw);
				}
				else
				{
					desiredHeading = currYaw;
				}

				if (modes[remotebit])
				{
					mode.request.mode = mode.request.CirclePoint;
					mode.request.surge = 0.5;

					mode.request.ref_point.header.frame_id = "worldLatLon";
					if ((data1 == 0) && (data2 == 0))
					{
						//Get current heading and calculate the desired point
						mode.request.ref_point.point.x = desiredLat;
						mode.request.ref_point.point.y = desiredLon;
					}
					else
					{
						mode.request.ref_point.point.x = data1/double(latlonmux);
						mode.request.ref_point.point.y = data2/double(latlonmux);
					}

					ROS_INFO("Switch to remote mode: %f %f.",
							mode.request.ref_point.point.x,
							mode.request.ref_point.point.y);
				}
				else
				{
					std::pair<double, double> location = labust::tools::meter2deg(buoyDistance*cos(currYaw),
							buoyDistance*sin(currYaw),
							currLat);
					//Get current heading and calculate the desired point
					desiredLat= currLat + location.first;
					desiredLon = currLon + location.second;
				}
			}

			boost::mutex::scoped_lock l2(clientMux);
			lastModemMsg = ros::Time::now();
			if (!client)
			{
				ROS_ERROR("HLManager client not connected. Trying reset.");
				client = nh.serviceClient<navcon_msgs::SetHLMode>("SetHLMode", true);
			}
			else
			{
				client.call(mode);
			}

			this->reply();
		}
	}
	else
	{
		ROS_ERROR("Wrong CRC.");
	}

	start_receive();
}

void CNRRemoteRadio::reply()
{
	char ret[17];
	ret[0] = 'C';
	ret[1] = 'P';
	ret[2] = 12;
	ret[3] = (station<<4) + cart;
	ret[4] = lastmode;
	boost::mutex::scoped_lock l(cdataMux);
	uint32_t lat = htonl(currLat*latlonmux);
	uint32_t lon = htonl(currLon*latlonmux);
	uint16_t hdg = htons(currYaw*100/M_PI*180);
	l.unlock();
	ROS_INFO("The cart reply: %d, %d %d",htonl(lat), htonl(lon), int16_t(htons(hdg)));
	memcpy(&ret[5],&lat,sizeof(uint32_t));
	memcpy(&ret[9],&lon,sizeof(uint32_t));
	memcpy(&ret[13],&hdg,sizeof(uint16_t));
	int crc = compute_crc16(&ret[0], 15);
	ret[15] = crc/256;
	ret[16] = crc%256;
	boost::asio::write(port, boost::asio::buffer(ret,sizeof(ret)));
}

void CNRRemoteRadio::replyBuoy()
{
	char ret[15];
	ret[0] = 'C';
	ret[1] = 'P';
	ret[2] = 10;
	ret[3] = (station<<4) + bart;
	boost::mutex::scoped_lock l(cdataMux);
	uint32_t lat = htonl(currLat*latlonmux);
	uint32_t lon = htonl(currLon*latlonmux);
	l.unlock();
	ROS_INFO("The buoy reply: %d, %d",htonl(lat), htonl(lon));
	memcpy(&ret[4],&lat,sizeof(uint32_t));
	memcpy(&ret[8],&lon,sizeof(uint32_t));
	ret[12] = doLaunch;
	int crc = compute_crc16(&ret[0], 13);
	ret[13] = crc/256;
	ret[14] = crc%256;
	boost::asio::write(port, boost::asio::buffer(ret,sizeof(ret)));
}

void CNRRemoteRadio::dummyRequest()
{
	char ret[7];
	ret[0] = 'C';
	ret[1] = 'P';
	ret[2] = 2;
	ret[3] = (bart<<4) + station;
	ret[4] = doLaunch;

	int crc = compute_crc16(&ret[0], 5);
	ret[5] = crc/256;
	ret[6] = crc%256;
	boost::asio::write(port, boost::asio::buffer(ret,sizeof(ret)));
}

void CNRRemoteRadio::onTimeout()
{
	if (!client)
	{
		ROS_ERROR("HLManager client not connected. Trying reset.");
		client = nh.serviceClient<navcon_msgs::SetHLMode>("SetHLMode", true);
	}
	else
	{
		navcon_msgs::SetHLMode mode;
		mode.request.mode = 0;
		client.call(mode);
	}

	ROS_ERROR("Lost connection with CNRRemote!");

	io.stop();
	iorunner.join();
	port.close();

	io.reset();
	lastModemMsg = ros::Time::now();
	ros::Rate rate(1);
	for(int i=0;i<3;++i) rate.sleep();

	std::string portName;
	int baud;
	ph.param("PortName",portName,portName);
	ph.param("BaudRate",baud,baud);

	port.open(portName);
	port.set_option(boost::asio::serial_port::baud_rate(baud));

	if (!port.is_open())
	{
		ROS_ERROR("Cannot open port.");
		throw std::runtime_error("Unable to open the port.");
	}

	this->start_receive();
	iorunner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
}

void CNRRemoteRadio::start()
{
	ros::Rate rate(20);

	ros::Time lastDummyReq(ros::Time::now());
	while (ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
		boost::mutex::scoped_lock l(clientMux);
		if ((ros::Time::now()-lastModemMsg).toSec() > timeout )
		{
			this->onTimeout();
		}

		if (doDummyRequest && ((ros::Time::now() - lastDummyReq).toSec() > 0.5))
		{
			lastDummyReq = ros::Time::now();
//			if (this->id == cart)
//				this->reply();
//			else
//				this->dummyRequest();
		}
	}
}

