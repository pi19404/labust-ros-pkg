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
#include <labust/tools/rosutils.hpp>
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
		ph("~"),
		lastModemMsg(ros::Time::now()),
		timeout(10),
		port(io),
		isTopside(true),
		twoWayComms(false)
{this->onInit();}

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
	ph.param("BaudRate",baud,baud);
	ph.param("IsTopside",isTopside,isTopside);
	ph.param("TwoWay",twoWayComms,twoWayComms);
	ph.param("Timeout",timeout,timeout);

	port.open(portName);
	port.set_option(boost::asio::serial_port::baud_rate(baud));

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

		stateHatPub = nh.advertise<auv_msgs::NavSts>("stateHat",1);
		stateMeasPub = nh.advertise<auv_msgs::NavSts>("meas",1);

		nh.param("LocalOriginLat",originLat,originLat);
		nh.param("LocalOriginLon",originLon,originLon);
	}
	else
	{
		joyOut = nh.advertise<sensor_msgs::Joy>("joy_out",1);
		launched = nh.advertise<std_msgs::Bool>("launched",1);
		hlMsg = nh.advertise<cart2::HLMessage>("hl_message",1);
		client = nh.serviceClient<cart2::SetHLMode>("SetHLMode", true);
		stateHat = nh.subscribe<auv_msgs::NavSts>("stateHat",1,&TopsideRadio::onStateHat,this);
		stateMeas = nh.subscribe<auv_msgs::NavSts>("meas",1,&TopsideRadio::onStateMeas,this);
		curMode = nh.subscribe<std_msgs::Int32>("current_mode",1,&TopsideRadio::onCurrentMode,this);
	}

	populateDataFromConfig();
	cdata.origin_lat = 0;
	cdata.origin_lon = 0;

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

void TopsideRadio::onCurrentMode(const std_msgs::Int32::ConstPtr& mode)
{
	boost::mutex::scoped_lock l(cdataMux);
	cdata.mode = mode->data;
}

void TopsideRadio::onStateHat(const auv_msgs::NavSts::ConstPtr& estimate)
{
	boost::mutex::scoped_lock l(cdataMux);
	cdata.state_hat[u] = estimate->body_velocity.x;
	cdata.state_hat[r] = estimate->orientation_rate.yaw;
	cdata.state_hat[x] = estimate->position.north;
	cdata.state_hat[y] = estimate->position.east;
	cdata.state_hat[psi] = estimate->orientation.yaw;

	try
	{
		tf::StampedTransform transformLocal;
		listener.lookupTransform("worldLatLon", "local", ros::Time(0), transformLocal);
		cdata.origin_lat = transformLocal.getOrigin().y();
		cdata.origin_lon = transformLocal.getOrigin().x();
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}
}

void TopsideRadio::onStateMeas(const auv_msgs::NavSts::ConstPtr& meas)
{
	boost::mutex::scoped_lock l(cdataMux);
	cdata.state_meas[u] = meas->body_velocity.x;
	cdata.state_meas[r] = meas->orientation_rate.yaw;
	cdata.state_meas[x] = meas->position.north;
	cdata.state_meas[y] = meas->position.east;
	cdata.state_meas[psi] = meas->orientation.yaw;
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
	data.mode_update = 1;
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
	data.yaw = config.Yaw;
}

void TopsideRadio::dynrec_cb(cart2::RadioModemConfig& config, uint32_t level)
{
	if ((config.OpMode != this->config.OpMode) &&
			(config.OpMode == 0))
	{
		//Prevent re-launching when stopped
		config.Launch = 0;
		server.updateConfig(config);
	}
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

		if (transferred == 1)
		{
			ringBuffer.push_back(sbuffer.sbumpc());
			if (ringBuffer.size() > sync_length)
			{
				ringBuffer.erase(ringBuffer.begin());
			}
		}
		else
		{
			std::istream is(&sbuffer);
			ringBuffer.clear();
			is >> ringBuffer;
		}

		if ((ringBuffer.size() >= sync_length) && (ringBuffer.substr(0,sync_length) == "@ONTOP"))
		{
			ROS_INFO("Synced on @ONTOP");
			assert(!isTopside && "Cannot receive topside messages if on topside.");
			boost::asio::async_read(port,sbuffer.prepare(topside_package_length),boost::bind(&TopsideRadio::onIncomingData,this,_1,_2));
		}
		else if ((ringBuffer.size() >= sync_length) && (ringBuffer.substr(0,sync_length) == "@CART2"))
		{
			ROS_INFO("Synced on @CART2");
			assert(isTopside && "Cannot receive CART messages if on cart.");
			boost::asio::async_read(port,sbuffer.prepare(cart_package_length),boost::bind(&TopsideRadio::onIncomingData,this,_1,_2));
		}
		else
		{
			ROS_INFO("No sync: %s",ringBuffer.c_str());
			boost::asio::async_read(port, sbuffer.prepare(1),
					boost::bind(&TopsideRadio::onSync,this,_1,_2));
		}
	}
}

void TopsideRadio::onIncomingData(const boost::system::error_code& error, const size_t& transferred)
{
	sbuffer.commit(transferred);

	if (!error)
	{
	boost::archive::binary_iarchive dataSer(sbuffer, boost::archive::no_header);
	ROS_INFO("Received:%d", transferred);

	ROS_INFO("Received %d bytes.",transferred);

	if (!isTopside)
	{
		boost::mutex::scoped_lock l(dataMux);
		uint8_t chksum(0);
		try
		{
			dataSer >> data >> chksum;
		}
		catch (std::exception& e)
		{
			ROS_ERROR("Exception while deserializing: %s",e.what());
		}

		uint8_t chksum_calc = calculateChecksum(data);
		if (chksum_calc != chksum)
		{
			ROS_ERROR("Wrong checksum! Got: %d, expected: %d",chksum, chksum_calc);
			return;
		}

		sensor_msgs::Joy joy;
		joy.axes.assign(6,0);
		//Sanity check
		if (fabs(data.surgeForce) > 1)
		{
			ROS_ERROR("Remote joystick force is above 1.");
			data.surgeForce = 0;
		}

		if (fabs(data.torqueForce) > 1)
		{
			ROS_ERROR("Remote joystick force is above 1.");
			data.torqueForce = 0;
		}

		joy.axes[1] = data.surgeForce;
		joy.axes[2] = data.torqueForce;

		std_msgs::Bool launcher;
		launcher.data = data.launch;


		//\todo Update this to some real constant
		if (data.mode > 5)
		{
			data.mode = 0;
			ROS_ERROR("Wrong mode.");
			return;
		}
		cart2::HLMessagePtr msg(new cart2::HLMessage());
		bool update = data.mode_update;
		msg->mode = data.mode;
		msg->radius = data.radius;
		msg->ref_point.header.stamp=ros::Time::now();
		msg->ref_point.header.frame_id="worldLatLon";
		msg->ref_point.point.x = data.lat;
		msg->ref_point.point.y = data.lon;
		msg->surge = data.surge;
		l.unlock();

		joyOut.publish(joy);
		if (update)
		{
			hlMsg.publish(msg);
			launched.publish(launcher);
		}

		boost::mutex::scoped_lock l2(clientMux);
		lastModemMsg = ros::Time::now();
		if (!client)
		{
			ROS_ERROR("HLManager client not connected. Trying reset.");
			client = nh.serviceClient<cart2::SetHLMode>("SetHLMode", true);
		}
		else if (update)
		{
			boost::mutex::scoped_lock l(dataMux);
			cart2::SetHLMode mode;
			mode.request.mode = data.mode;
			mode.request.ref_point.header.stamp=ros::Time::now();
			mode.request.ref_point.header.frame_id="worldLatLon";
			mode.request.ref_point.point.x = data.lat;
			mode.request.ref_point.point.y = data.lon;
			mode.request.radius = data.radius;
			mode.request.surge = data.surge;
			mode.request.yaw = data.yaw;
			l.unlock();
			client.call(mode);
		}
		l2.unlock();

		if (twoWayComms)
		{
			boost::asio::streambuf output;
			std::ostream out(&output);
			//Prepare sync header
			out<<"@CART2";
			boost::archive::binary_oarchive dataSer(output, boost::archive::no_header);
			boost::mutex::scoped_lock l(cdataMux);
			uint8_t chksum_calc = calculateChecksum(cdata);
			try
			{
				dataSer << cdata << chksum_calc;
			}
			catch (std::exception& e)
			{
				ROS_ERROR("Exception while serializing: %s",e.what());
			}
			l.unlock();
			//write data
			int n = boost::asio::write(port, output.data());
			ROS_INFO("Transferred:%d",n);
		}
	}
	else
	{
		boost::mutex::scoped_lock l(cdataMux);
		uint8_t chksum(0);
		try
		{
			dataSer >> cdata >> chksum;
		}
		catch (std::exception& e)
		{
			ROS_ERROR("Exception while serializing: %s",e.what());
		}

		uint8_t chksum_calc = calculateChecksum(cdata);
		if (chksum_calc != chksum)
		{
			ROS_ERROR("Wrong checksum! Got: %d, expected: %d",chksum, chksum_calc);
			return;
		}

		auv_msgs::NavStsPtr state(new auv_msgs::NavSts());
		auv_msgs::NavStsPtr meas(new auv_msgs::NavSts());
		state->origin.latitude = cdata.origin_lat;
		state->origin.longitude = cdata.origin_lon;
		state->body_velocity.x = cdata.state_hat[u];
		state->orientation_rate.yaw = cdata.state_hat[r];
		state->position.north = cdata.state_hat[x];
		state->position.east = cdata.state_hat[y];
		state->orientation.yaw = cdata.state_hat[psi];
		meas->origin.latitude = cdata.origin_lat;
		meas->origin.longitude = cdata.origin_lon;
		meas->body_velocity.x = cdata.state_meas[u];
		meas->orientation_rate.yaw = cdata.state_meas[r];
		meas->position.north = cdata.state_meas[x];
		meas->position.east = cdata.state_meas[y];
		meas->orientation.yaw = cdata.state_meas[psi];

		ROS_INFO("Current CART2 mode:%d",cdata.mode);

		tf::Transform transform;
		transform.setOrigin(tf::Vector3(cdata.origin_lon, cdata.origin_lat, 0));
		l.unlock();

		transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "worldLatLon", "world"));
		transform.setOrigin(tf::Vector3(0, 0, 0));
		Eigen::Quaternion<float> q;
		labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,q);
		transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "local"));
		transform.setOrigin(tf::Vector3(state->position.north, state->position.east, 0));
		labust::tools::quaternionFromEulerZYX(0,0,state->orientation.yaw,q);
		transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local", "base_link"));
		transform.setOrigin(tf::Vector3(meas->position.north, meas->position.east, 0));
		labust::tools::quaternionFromEulerZYX(0,0,meas->orientation.yaw,q);
		transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local", "base_link_meas"));

		stateHatPub.publish(state);
		stateMeasPub.publish(meas);
	}
	}
	else
	{
		ROS_ERROR("Communication failed.");
	}

	start_receive();
}

void TopsideRadio::onTimeout()
{
	if (!client)
	{
		ROS_ERROR("HLManager client not connected. Trying reset.");
		client = nh.serviceClient<cart2::SetHLMode>("SetHLMode", true);
	}
	else
	{
		cart2::SetHLMode mode;
		mode.request.mode = 0;
		client.call(mode);
	}

	ROS_ERROR("Lost connection with topside!");

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

void TopsideRadio::start()
{
	if (!isTopside)
	{
		ros::Rate rate(20);

		while (ros::ok())
		{
			rate.sleep();
			ros::spinOnce();
			boost::mutex::scoped_lock l(clientMux);
			if ((ros::Time::now()-lastModemMsg).toSec() > timeout )
			{
				this->onTimeout();
			}
		}
	}
	else
	{
		double transmitRate(10);
		ros::NodeHandle ph("~");
		ph.param("TransmitRate",transmitRate,transmitRate);
		ros::Rate rate(transmitRate);
		while (nh.ok())
		{
			boost::asio::streambuf output;
			std::ostream out(&output);
			std::ostringstream chk;
			//Prepare sync header
			out<<"@ONTOP";
			boost::archive::binary_oarchive dataSer(output, boost::archive::no_header);
			boost::mutex::scoped_lock l(dataMux);
			uint8_t chksum_calc = calculateChecksum(data);
			try
			{
				dataSer << data << chksum_calc;
			}
			catch (std::exception& e)
			{
				ROS_ERROR("Exception while serializing: %s",e.what());
			}
			data.mode_update = 0;
			l.unlock();

			//write data
			int n = boost::asio::write(port, output.data());
			ROS_INFO("Transferred %d bytes.",n);

			rate.sleep();
			ros::spinOnce();
		}
	}
}

