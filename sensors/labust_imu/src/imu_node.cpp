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
#include <labust/tools/conversions.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/math/NumberManipulation.hpp>
//#include <cart2/ImuInfo.h>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <numeric>

struct SharedData
{
	enum {msg_size = 86,
		data_offset=4,
		checksum = 85,
		float_offset=9};

	enum {imuPos=0,
		gpsPos};
	enum {lat=0, lon=1};
	ros::Publisher imuPub, gpsPub, imuinfo;
	std::vector<geometry_msgs::TransformStamped> transforms;
	tf2_ros::TransformBroadcaster broadcast;
	double magnetic_declination;
	unsigned char buffer[msg_size];
	std::vector<double> median[2];
	int gps_pub;
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
		std::cout<<"Processing."<<std::endl;
		unsigned char calc = 0;
		for (size_t i=SharedData::data_offset; i<SharedData::msg_size-1; ++i){calc^=shared.buffer[i];};

		if (calc != shared.buffer[SharedData::checksum])
		{
			ROS_ERROR("Wrong checksum for imu data.");
			start_receive(shared,port);
			return;
		}

		float* data(reinterpret_cast<float*>(&shared.buffer[SharedData::data_offset + SharedData::float_offset]));
		enum {sog=0, cog, declination,
			accel_x, accel_y, accel_z,
			gyro_x, gyro_y, gyro_z,
			mag_x, mag_y, mag_z,
			roll,pitch,yaw,ry,mmm,mm};

		std_msgs::Float32MultiArray info;
		info.data.resize(mm+6);
		for (size_t i=0; i<mm+1; ++i) info.data[i+5] = data[i];

		//std::cout<<"Euler:"<<data[roll]<<","<<data[pitch]<<","<<data[yaw]<<std::endl;
		//std::cout<<"Magnetski:"<<data[mag_x]<<","<<data[mag_y]<<","<<data[mag_z]<<std::endl;
		//std::cout<<"Test:"<<","<<data[ry]<<","<<data[mmm]<<","<<data[mm]<<std::endl;

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
				labust::math::wrapRad(data[yaw] + shared.magnetic_declination), quat);
		imu->orientation.x = quat.x();
		imu->orientation.y = quat.y();
		imu->orientation.z = quat.z();
		imu->orientation.w = quat.w();
		shared.transforms[SharedData::imuPos].header.stamp = ros::Time::now();
		shared.broadcast.sendTransform(shared.transforms[SharedData::imuPos]);
		shared.imuPub.publish(imu);

		//Send GPS stuff
		sensor_msgs::NavSatFix::Ptr gps(new sensor_msgs::NavSatFix());

		int16_t* latlon(reinterpret_cast<int16_t*>(&shared.buffer[SharedData::data_offset]));
		enum{lat=0, fraclat,lon,fraclon};
		info.data[0] = latlon[lat];
		info.data[1] = latlon[fraclat];
		info.data[2] = latlon[lon];
		info.data[3] = latlon[fraclon];
		char status = info.data[4] = shared.buffer[SharedData::data_offset+SharedData::float_offset-1];
		shared.imuinfo.publish(info);
		gps->latitude = latlon[lat]/100 + (latlon[lat]%100 + latlon[fraclat]/10000.)/60. ;
		gps->longitude = latlon[lon]/100 + (latlon[lon]%100 + latlon[fraclon]/10000.)/60.;
		gps->position_covariance[0] = 9999;
		gps->position_covariance[4] = 9999;
		gps->position_covariance[8] = 9999;
		gps->header.frame_id = "worldLatLon";
		gps->header.stamp = ros::Time::now();
		shared.transforms[SharedData::gpsPos].header.stamp = ros::Time::now();
		shared.broadcast.sendTransform(shared.transforms[SharedData::gpsPos]);
		static int i=0;
		++i;
//		if ((status == 'A') && ((i%shared.gps_pub)==0)) shared.gpsPub.publish(gps);
		if (status == 'A')
		{
			shared.median[SharedData::lat].push_back(gps->latitude);
			shared.median[SharedData::lon].push_back(gps->longitude);
			if ((i%shared.gps_pub)==0)
			{
				double med[2];
				for (int i=0; i<2; ++i)
				{
					int size = shared.median[i].size();
					//Median
//					std::sort(shared.median[i].begin(),shared.median[i].end());
//					if (size%2)
//					{
//						med[i] = shared.median[i][size/2-1];
//					}
//					else
//					{
//						med[i] = (shared.median[i][size/2] +
//								shared.median[i][size/2-1])/2;
//					}
//
					//Average
					med[i] = std::accumulate(shared.median[i].begin(), shared.median[i].end(), 0.0)/size;
					shared.median[i].clear();
				};

				gps->latitude = med[SharedData::lat];
				gps->longitude = med[SharedData::lon];
				shared.gpsPub.publish(gps);
			}
		}
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
	ph.param("gps_pub",shared.gps_pub,1);
	shared.imuPub = nh.advertise<sensor_msgs::Imu>("imu",1);
	shared.gpsPub = nh.advertise<sensor_msgs::NavSatFix>("fix",1);
	shared.imuinfo= nh.advertise<std_msgs::Float32MultiArray>("imu_info",1);

	//Configure Imu and GPS position relative to vehicle center of mass
	Eigen::Vector3d origin(Eigen::Vector3d::Zero()),
			orientation(Eigen::Vector3d::Zero());

	labust::tools::getMatrixParam(nh, "imu_origin", origin);
	labust::tools::getMatrixParam(nh, "imu_orientation", orientation);
	shared.transforms[SharedData::imuPos].transform.translation.x = origin(0);
	shared.transforms[SharedData::imuPos].transform.translation.y = origin(1);
	shared.transforms[SharedData::imuPos].transform.translation.z = origin(2);
	labust::tools::quaternionFromEulerZYX(orientation(0),
			orientation(1),
			orientation(2),
			shared.transforms[SharedData::imuPos].transform.rotation);
	shared.transforms[SharedData::imuPos].child_frame_id = "imu_frame";
	shared.transforms[SharedData::imuPos].header.frame_id = "base_link";
	shared.transforms[SharedData::imuPos].header.stamp = ros::Time::now();

	origin = origin.Zero();
	orientation = orientation.Zero();
	labust::tools::getMatrixParam(nh, "gps_origin", origin);
	labust::tools::getMatrixParam(nh, "gps_orientation", orientation);
	shared.transforms[SharedData::gpsPos].transform.translation.x = origin(0);
	shared.transforms[SharedData::gpsPos].transform.translation.y = origin(1);
	shared.transforms[SharedData::gpsPos].transform.translation.z = origin(2);
	labust::tools::quaternionFromEulerZYX(orientation(0),
			orientation(1),
			orientation(2),
			shared.transforms[SharedData::gpsPos].transform.rotation);
	shared.transforms[SharedData::gpsPos].child_frame_id = "imu_frame";
	shared.transforms[SharedData::gpsPos].header.frame_id = "base_link";
	shared.transforms[SharedData::gpsPos].header.stamp = ros::Time::now();

	start_receive(shared,port);
	boost::thread t(boost::bind(&ba::io_service::run,&io));

	ros::spin();
	io.stop();
	t.join();

	return 0;
}


