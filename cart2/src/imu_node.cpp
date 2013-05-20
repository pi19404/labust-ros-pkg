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
#include <labust/tools/rosutils.hpp>
#include <cart2/ImuInfo.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <iostream>

struct SharedData
{
	enum {msg_size = 85,
		data_offset=4,
		checksum = 84};
	ros::Publisher imuPub, gpsPub, imuinfo;
	tf::Transform imuPos, gpsPos, worldLatLon, world;
	tf::TransformBroadcaster broadcast;
	double magnetic_declination;
	unsigned char buffer[msg_size];
};

void start_receive(SharedData& shared,
		boost::asio::serial_port& port, bool single=false);
void handleIncoming(SharedData& shared,
		boost::asio::serial_port& port,
		const boost::system::error_code& error, const size_t& transferred);

void sync(SharedData& shared,
		boost::asio::serial_port& port,
		const boost::system::error_code& error, const size_t& transferred)
{
	if (!error)
	{
		bool flag = true;
		for(int i=0; i<SharedData::data_offset;++i) flag = flag && (shared.buffer[i] == 0xFF);

		if (flag)
		{
			std::cout<<"Sync."<<std::endl;
			boost::asio::async_read(port, boost::asio::buffer(&shared.buffer[SharedData::data_offset],
					SharedData::msg_size-SharedData::data_offset),
					boost::bind(&handleIncoming,
							boost::ref(shared),
							boost::ref(port),_1,_2));
		}
		else
		{
			//Rotate buffer to the left and read the next byte on the end.
			for(int i=0; i<SharedData::data_offset-1;++i) shared.buffer[i] = shared.buffer[i+1];
			std::cout<<"No sync."<<std::endl;
			start_receive(shared,port,true);
		}
	}
}

void handleIncoming(SharedData& shared,
		boost::asio::serial_port& port,
		const boost::system::error_code& error, const size_t& transferred)
{
	std::cout<<"Got stuff."<<std::endl;
	if (!error && (transferred == (SharedData::msg_size-SharedData::data_offset)))
	{
		unsigned char calc = 0;
		for (size_t i=SharedData::data_offset; i<SharedData::msg_size-1; ++i){calc^=shared.buffer[i];};

		if (calc != shared.buffer[SharedData::checksum])
		{
			ROS_ERROR("Wrong checksum for imu data.");
			//return;
		}

		float* data(reinterpret_cast<float*>(&shared.buffer[SharedData::data_offset]));
		enum {time = 0,
			lat, lon, hdop,
			accel_x, accel_y, accel_z,
			gyro_x, gyro_y, gyro_z,
			mag_x, mag_y, mag_z,
			roll,pitch,yaw,
			modul,ry,mmm,mm};

		cart2::ImuInfo info;
		info.data.resize(mm+1);
		for (size_t i=0; i<mm+1; ++i) info.data[i] = data[i];
		shared.imuinfo.publish(info);

		//std::cout<<"Euler:"<<data[roll]<<","<<data[pitch]<<","<<data[yaw]<<std::endl;
		//std::cout<<"Magnetski:"<<data[mag_x]<<","<<data[mag_y]<<","<<data[mag_z]<<std::endl;
		//std::cout<<"Test:"<<data[modul]<<","<<data[ry]<<","<<data[mmm]<<","<<data[mm]<<std::endl;

		//Send Imu stuff
		sensor_msgs::Imu::Ptr imu(new sensor_msgs::Imu());
		imu->header.stamp = ros::Time::now();
		imu->header.frame_id = "imu_frame";
		imu->linear_acceleration.x = data[accel_x];
		imu->linear_acceleration.y = data[accel_y];
		imu->linear_acceleration.z = data[accel_z];
		imu->angular_velocity.x = data[gyro_x];
		imu->angular_velocity.y = data[gyro_y];
		imu->angular_velocity.z = data[gyro_z];

		Eigen::Quaternion<float> quat;
		labust::tools::quaternionFromEulerZYX(data[roll],
				data[pitch],
				data[yaw] + shared.magnetic_declination, quat);
		imu->orientation.x = quat.x();
		imu->orientation.y = quat.y();
		imu->orientation.z = quat.z();
		imu->orientation.w = quat.w();
		shared.broadcast.sendTransform(tf::StampedTransform(shared.imuPos, ros::Time::now(), "base_link", "imu_frame"));
		shared.imuPub.publish(imu);

		//Send GPS stuff
		sensor_msgs::NavSatFix::Ptr gps(new sensor_msgs::NavSatFix());

		int latDeg(data[lat]/100), lonDeg(data[lon]/100);
		gps->latitude = latDeg + (data[lat]/100-latDeg)/0.6;
		gps->longitude = lonDeg + (data[lon]/100-lonDeg)/0.6;
		gps->position_covariance[0] = data[hdop];
		gps->position_covariance[4] = data[hdop];
		gps->position_covariance[8] = 9999;
		gps->header.frame_id = "worldLatLon";
		gps->header.stamp = ros::Time::now();
		shared.broadcast.sendTransform(tf::StampedTransform(shared.gpsPos, ros::Time::now(), "base_link", "gps_frame"));
		shared.gpsPub.publish(gps);

		//Send the WorldLatLon frame update
		shared.broadcast.sendTransform(tf::StampedTransform(shared.worldLatLon, ros::Time::now(), "worldLatLon", "world"));
		shared.broadcast.sendTransform(tf::StampedTransform(shared.world, ros::Time::now(), "world", "local"));
	}
	start_receive(shared,port);
}

void start_receive(SharedData& shared,
		boost::asio::serial_port& port, bool single)
{
	if (single)
	{
		boost::asio::async_read(port, boost::asio::buffer(&shared.buffer[SharedData::data_offset-1],1),
				boost::bind(&sync,
						boost::ref(shared),
						boost::ref(port),_1,_2));
	}
	else
	{
		boost::asio::async_read(port, boost::asio::buffer(&shared.buffer[0],SharedData::data_offset),
				boost::bind(&sync,
						boost::ref(shared),
						boost::ref(port),_1,_2));
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"imu_node");
	ros::NodeHandle nh,ph("~");

	std::string portName("/dev/ttyUSB0");
	int baud(115200);

	ph.param("PortName",portName,portName);
	ph.param("Baud",baud,baud);

	namespace ba=boost::asio;
	ba::io_service io;
	ba::serial_port port(io);

	port.open(portName);
	port.set_option(ba::serial_port::baud_rate(baud));
	port.set_option(ba::serial_port::flow_control(
			ba::serial_port::flow_control::none));

	if (!port.is_open())
	{
		std::cerr<<"Cannot open port."<<std::endl;
		exit(-1);
	}

	SharedData shared;
	ph.param("magnetic_declination",shared.magnetic_declination,0.0);
	shared.imuPub = nh.advertise<sensor_msgs::Imu>("imu",1);
	shared.gpsPub = nh.advertise<sensor_msgs::NavSatFix>("fix",1);
	shared.imuinfo= nh.advertise<cart2::ImuInfo>("imu_info",1);

	//Configure Imu and GPS position relative to vehicle center of mass
	Eigen::Vector3d origin(Eigen::Vector3d::Zero()),
			orientation(Eigen::Vector3d::Zero());

	labust::tools::getMatrixParam(nh, "imu_origin", origin);
	labust::tools::getMatrixParam(nh, "imu_orientation", orientation);
	shared.imuPos.setOrigin(tf::Vector3(origin(0),origin(1),origin(2)));
	shared.imuPos.setRotation(tf::createQuaternionFromRPY(orientation(0),
			orientation(1),orientation(2)));

	origin = origin.Zero();
	orientation = orientation.Zero();
	labust::tools::getMatrixParam(nh, "gps_origin", origin);
	labust::tools::getMatrixParam(nh, "gps_orientation", orientation);
	shared.gpsPos.setOrigin(tf::Vector3(origin(0),origin(1),origin(2)));
	shared.gpsPos.setRotation(tf::createQuaternionFromRPY(orientation(0),
			orientation(1),orientation(2)));

	//Setup the world coordinates
	double originLat(0), originLon(0);
	nh.param("LocalOriginLat",originLat,originLat);
	nh.param("LocalOriginLon",originLon,originLon);
	shared.worldLatLon.setOrigin(tf::Vector3(originLon, originLat, 0));
	shared.worldLatLon.setRotation(tf::createQuaternionFromRPY(0,0,0));
	Eigen::Quaternion<float> q;
	labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,q);
	shared.world.setOrigin(tf::Vector3(0,0,0));
	shared.world.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));

	start_receive(shared,port);
	boost::thread t(boost::bind(&ba::io_service::run,&io));

	ros::spin();
	io.stop();
	t.join();

	return 0;
}


