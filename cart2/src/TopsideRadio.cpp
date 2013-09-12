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
#include <labust/tools/conversions.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <cart2/SetHLMode.h>
#include <cart2/HLMessage.h>
#include <cmath>

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
								twoWayComms(false),
								lastMode(0)
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
		extPoint = nh.subscribe<auv_msgs::NavSts>("target_point", 1,
				&TopsideRadio::onExtPoint,this);
		joyIn = nh.subscribe<sensor_msgs::Joy>("joy_in",1,&TopsideRadio::onJoy,this);
		stateHatPub = nh.advertise<auv_msgs::NavSts>("stateHat",1);
		stateMeasPub = nh.advertise<auv_msgs::NavSts>("meas",1);
		info = nh.advertise<cart2::ImuInfo>("cart2_info",1);
		selectedPoint = nh.advertise<geometry_msgs::PointStamped>("selected_point", 1);
		selectedNavSts = nh.advertise<auv_msgs::NavSts>("selected_navsts", 1);

		//Dynamic reconfigure
		server.setCallback(boost::bind(&TopsideRadio::dynrec_cb, this, _1, _2));

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
		sfFrame = nh.subscribe<auv_msgs::NavSts>("sf_diagnostics",1,&TopsideRadio::onSFMeas,this);
		curMode = nh.subscribe<std_msgs::Int32>("current_mode",1,&TopsideRadio::onCurrentMode,this);
		cartInfo = nh.subscribe<cart2::ImuInfo>("cart2_info",1,&TopsideRadio::onCartInfo,this);
	}

	populateDataFromConfig();
	cdata.origin_lat = 0;
	cdata.origin_lon = 0;
	cdata.mode = 0;
	cdata.sf_state[x]= cdata.sf_state[y]= cdata.sf_state[psi]= 0;
	data.mode = 0;
	data.surgeForce = data.torqueForce = 0;

	this->start_receive();
	iorunner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
}

void TopsideRadio::onJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
	boost::mutex::scoped_lock l(dataMux);
	data.surgeForce = joy->axes[1]*100;
	data.torqueForce = joy->axes[2]*100;
}

void TopsideRadio::onExtPoint(const auv_msgs::NavSts::ConstPtr& extPoint)
{
	if (!config.ManualPoint)
	{
		boost::mutex::scoped_lock l(dataMux);
		if (extPoint->header.frame_id == "worldLatLon")
		{
			data.lat = extPoint->global_position.latitude*10000000;
			data.lon = extPoint->global_position.longitude*10000000;
		}
		else if (extPoint->header.frame_id == "local")
		{
			local2LatLon(extPoint->position.north, extPoint->position.east);
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
	lastMode = mode->data;
}

void TopsideRadio::onStateHat(const auv_msgs::NavSts::ConstPtr& estimate)
{
	boost::mutex::scoped_lock l(cdataMux);
	cdata.state_hat[u] = estimate->body_velocity.x*100;
	//cdata.state_hat[r] = estimate->orientation_rate.yaw*100;
	cdata.state_hat[x] = estimate->position.north*100;
	cdata.state_hat[y] = estimate->position.east*100;
	cdata.state_hat[psi] = estimate->orientation.yaw*100;

	try
	{
		tf::StampedTransform transformLocal;
		listener.lookupTransform("/worldLatLon", "local", ros::Time(0), transformLocal);
		cdata.origin_lat = transformLocal.getOrigin().y()*10000000;
		cdata.origin_lon = transformLocal.getOrigin().x()*10000000;
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
	}
}

void TopsideRadio::onStateMeas(const auv_msgs::NavSts::ConstPtr& meas)
{
	boost::mutex::scoped_lock l(cdataMux);
	//cdata.state_meas[u] = meas->body_velocity.x*100;
	//cdata.state_meas[r] = meas->orientation_rate.yaw*100;
	cdata.state_meas[x] = meas->position.north*100;
	cdata.state_meas[y] = meas->position.east*100;
	cdata.state_meas[psi] = meas->orientation.yaw*100;
}

void TopsideRadio::onSFMeas(const auv_msgs::NavSts::ConstPtr& meas)
{
	boost::mutex::scoped_lock l(cdataMux);
	//cdata.state_meas[u] = meas->body_velocity.x*100;
	//cdata.state_meas[r] = meas->orientation_rate.yaw*100;
	cdata.sf_state[x] = meas->position.north*100;
	cdata.sf_state[y] = meas->position.east*100;
	cdata.sf_state[psi] = meas->orientation.yaw*100;
}

void TopsideRadio::onCartInfo(const cart2::ImuInfo::ConstPtr& info)
{
	boost::mutex::scoped_lock l(cdataMux);
	enum {port_rpm_desired=0,
		stbd_rpm_desired,
		port_rpm_meas,
		stbd_rpm_meas,
		port_curr_desired,
		stbd_curr_desired,
		current,
		temp,
		voltage
	};

	cdata.portRPM = info->data[port_rpm_meas];
	cdata.stbdRPM = info->data[stbd_rpm_meas];
	cdata.voltage = info->data[voltage]/50*256;
	cdata.temp = info->data[temp]/100*256;
}

void TopsideRadio::local2LatLon(double x, double y)
{
	std::pair<double, double> location = labust::tools::meter2deg(x,
			y,
			originLat);
	data.lat = (originLat + location.first)*10000000;
	data.lon = (originLon + location.second)*10000000;
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
			data.lat = config.PointLat*10000000;
			data.lon = config.PointLon*10000000;
		}
	}
	data.radius = config.Radius;
	data.surge = config.Surge*100;
	data.yaw = std::floor(config.Yaw/M_PI*128+0.5);
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

	if (config.ManualPoint && config.UseLocal)
	{
		geometry_msgs::PointStamped point;
		point.header.frame_id = "local";
		point.header.stamp = ros::Time::now();
		point.point.x = this->config.PointN;
		point.point.y = this->config.PointE;
		selectedPoint.publish(point);
	}
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

		if ((ringBuffer.size() >= sync_length) && (ringBuffer.substr(0,sync_length) == "@T"))
		{
			ROS_INFO("Synced on @ONTOP");
			assert(!isTopside && "Cannot receive topside messages if on topside.");
			boost::asio::async_read(port,sbuffer.prepare(topside_package_length),boost::bind(&TopsideRadio::onIncomingData,this,_1,_2));
		}
		else if ((ringBuffer.size() >= sync_length) && (ringBuffer.substr(0,sync_length) == "@C"))
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
			uint16_t chksum(0);
			try
			{
				dataSer >> data >> chksum;
			}
			catch (std::exception& e)
			{
				ROS_ERROR("Exception while deserializing: %s",e.what());
			}

			uint16_t chksum_calc = calculateCRC16(data);
			if (chksum_calc != chksum)
			{
				ROS_ERROR("Wrong checksum! Got: %d, expected: %d",chksum, chksum_calc);
				start_receive();
				return;
			}

			sensor_msgs::Joy joy;
			joy.axes.assign(6,0);
			//Sanity check
			if (fabs(data.surgeForce) > 100)
			{
				ROS_ERROR("Remote joystick force is above 1.");
				data.surgeForce = 0;
			}

			if (fabs(data.torqueForce) > 100)
			{
				ROS_ERROR("Remote joystick force is above 1.");
				data.torqueForce = 0;
			}

			joy.axes[1] = data.surgeForce/100.;
			joy.axes[2] = data.torqueForce/100.;

			std_msgs::Bool launcher;
			launcher.data = data.launch;


			//\todo Update this to some real constant
			if (data.mode > 8)
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
			msg->ref_point.point.x = data.lat/10000000.;
			msg->ref_point.point.y = data.lon/10000000.;
			msg->surge = data.surge/100.;
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
			else
			{
				boost::mutex::scoped_lock l(dataMux);
				cart2::SetHLMode mode;
				if (update)
				{
					lastMode = mode.request.mode = data.mode;
				}
				else
				{
					mode.request.mode = lastMode;
				}
				mode.request.ref_point.header.stamp=ros::Time::now();
				mode.request.ref_point.header.frame_id="worldLatLon";
				mode.request.ref_point.point.x = data.lat/10000000.;
				mode.request.ref_point.point.y = data.lon/10000000.;
				mode.request.radius = data.radius;
				mode.request.surge = data.surge/100.;
				mode.request.yaw = M_PI*data.yaw/128.;
				l.unlock();
				if (!(data.launch && (data.mode == 0))) client.call(mode);
			}
			l2.unlock();

			if (twoWayComms)
			{
				boost::asio::streambuf output;
				std::ostream out(&output);
				//Prepare sync header
				out<<"@C";
				boost::archive::binary_oarchive dataSer(output, boost::archive::no_header);
				boost::mutex::scoped_lock l(cdataMux);
				uint16_t chksum_calc = calculateCRC16(cdata);
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
			uint16_t chksum(0);
			try
			{
				dataSer >> cdata >> chksum;
			}
			catch (std::exception& e)
			{
				ROS_ERROR("Exception while serializing: %s",e.what());
			}

			uint16_t chksum_calc = calculateCRC16(cdata);
			if (chksum_calc != chksum)
			{
				ROS_ERROR("Wrong checksum! Got: %d, expected: %d",chksum, chksum_calc);
				start_receive();
				return;
			}

			auv_msgs::NavStsPtr state(new auv_msgs::NavSts());
			auv_msgs::NavStsPtr meas(new auv_msgs::NavSts());
			state->origin.latitude = cdata.origin_lat/10000000.;
			state->origin.longitude = cdata.origin_lon/10000000.;
			state->body_velocity.x = cdata.state_hat[u]/100.;
			//state->orientation_rate.yaw = cdata.state_hat[r]/100.;
			state->position.north = cdata.state_hat[x]/100.;
			state->position.east = cdata.state_hat[y]/100.;
			state->orientation.yaw = cdata.state_hat[psi]/100.;
			meas->origin.latitude = cdata.origin_lat/10000000.;
			meas->origin.longitude = cdata.origin_lon/10000000.;
			//meas->body_velocity.x = cdata.state_meas[u]/100.;
			//meas->orientation_rate.yaw = cdata.state_meas[r]/100.;
			meas->position.north = cdata.state_meas[x]/100.;
			meas->position.east = cdata.state_meas[y]/100.;
			meas->orientation.yaw = cdata.state_meas[psi]/100.;

			std::pair<double, double> location = labust::tools::meter2deg(
					state->position.north,
					state->position.east,
					state->origin.latitude);
			state->global_position.latitude = state->origin.latitude + location.first;
			state->global_position.longitude = state->origin.longitude + location.second;

			location = labust::tools::meter2deg(
					meas->position.north,
					meas->position.east,
					meas->origin.latitude);
			meas->global_position.latitude = state->origin.latitude + location.first;
			meas->global_position.longitude = state->origin.longitude + location.second;

			cart2::ImuInfo cinfo;
			cinfo.data.resize(4);
			cinfo.data[0] = cdata.voltage/256.*50;
			cinfo.data[1] = cdata.temp/256.*100;
			cinfo.data[2] = cdata.portRPM;
			cinfo.data[3] = cdata.stbdRPM;
			info.publish(cinfo);

			ROS_INFO("Current CART2 mode:%d",cdata.mode);

			tf::Transform transform;
			transform.setOrigin(tf::Vector3(cdata.origin_lon/10000000., cdata.origin_lat/10000000., 0));
			originLon = cdata.origin_lon/10000000.;
			originLat = cdata.origin_lat/10000000.;
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
			transform.setOrigin(tf::Vector3(cdata.sf_state[x]/100.,cdata.sf_state[y]/100., 0));
			labust::tools::quaternionFromEulerZYX(0,0,cdata.sf_state[psi]/100.,q);
			transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
			broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local", "sf_frame"));

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
			out<<"@T";
			boost::archive::binary_oarchive dataSer(output, boost::archive::no_header);
			boost::mutex::scoped_lock l(dataMux);
			uint16_t chksum_calc = calculateCRC16(data);
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

			if (config.OpMode == 7)
			{
				auv_msgs::NavSts selected;
				boost::mutex::scoped_lock l(cdataMux);
				std::pair<double, double> location = labust::tools::meter2deg(cdata.sf_state[x]/100.,
						cdata.sf_state[y]/100.,
						originLat);
				selected.global_position.latitude = originLat + location.first;
				selected.global_position.longitude = originLon + location.second;
				selected.position.north = cdata.sf_state[x]/100.;
				selected.position.east = cdata.sf_state[y]/100.;
				selected.orientation.yaw = cdata.sf_state[psi]/100.;
				selectedNavSts.publish(selected);

				geometry_msgs::PointStamped point;
				point.header.frame_id = "local";
				point.header.stamp = ros::Time::now();
				point.point.x = cdata.sf_state[x]/100.;
				point.point.y = cdata.sf_state[y]/100.;
				selectedPoint.publish(point);

			}

			if ((config.OpMode != 7) && config.ManualPoint)
			{
				auv_msgs::NavSts selected;
				if (config.UseLocal)
				{
					std::pair<double, double> location = labust::tools::meter2deg(config.PointN,
							config.PointE,
							originLat);
					selected.global_position.latitude = originLat + location.first;
					selected.global_position.longitude = originLon + location.second;
					selected.position.north = config.PointN;
					selected.position.east = config.PointE;
				}
				else
				{
					selected.global_position.latitude = config.PointLat;
					selected.global_position.longitude = config.PointLon;
					std::pair<double, double> location = labust::tools::deg2meter(config.PointLat - originLat,
							config.PointLon - originLon,
							originLat);
					selected.position.north = location.first;
					selected.position.east = location.second;
				}
				selectedNavSts.publish(selected);
			}


			rate.sleep();
			ros::spinOnce();
		}
	}
}

