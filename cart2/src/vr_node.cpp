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
*  Created: 01.02.2013.
*********************************************************************/
#include <labust/vehicles/VR3Details.hpp>
#include <labust/vehicles/Allocation.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/rosutils.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>
#include <auv_msgs/BodyForceReq.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>

#include <string>

struct SharedData
{
	enum {revLimit = 220, lightLimit=200};

	SharedData():port(io){};

	void portOpen(const std::string& portName, int baud)
	{
		port.open(portName);

		if (port.is_open())
		{
			using namespace boost::asio;
			port.set_option(serial_port::baud_rate(baud));
			port.set_option(serial_port::flow_control(serial_port::flow_control::none));
			port.set_option(serial_port::parity(serial_port::parity::none));
			port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
			port.set_option(serial_port::character_size(8));
			ROS_INFO("Port %s is open.", portName.c_str());
		}
	}

	ros::Publisher tauAch, imu, depth;
	auv_msgs::BodyForceReq tau;
	/**
	 * Thruster affine mappings.
	 */
	double Tnn,_Tnn;

	boost::asio::io_service io;
	boost::asio::serial_port port;

	labust::vehicles::VRSerialComms comms;
	labust::vehicles::VRComms::ThrustVec thrusters;
};

inline double getRevs(double thrust, double Tnn = 1, double _Tnn = 1)
{
	return (thrust >= 0) ? std::sqrt(thrust/Tnn) : -std::sqrt(-thrust/_Tnn);
}

void onTau(SharedData& shared, const auv_msgs::BodyForceReq::ConstPtr tau)
{
	double port, stbd;
	using namespace labust::vehicles;
	using namespace labust::math;
	port = getRevs(
			(tau->wrench.force.x + tau->wrench.torque.z)/2,
			shared.Tnn,
			shared._Tnn);
	stbd = getRevs(
			(tau->wrench.force.x - tau->wrench.torque.z)/2,
			shared.Tnn,
			shared._Tnn);
	shared.thrusters[VRComms::vert] = -coerce(AffineThruster::getRevs(
			tau->wrench.force.z,
			shared.Tnn,
			shared._Tnn), -SharedData::revLimit, SharedData::revLimit);

	double scale(1);
	if (port/SharedData::revLimit > scale) scale = port/SharedData::revLimit;
	if (stbd/SharedData::revLimit > scale) scale = stbd/SharedData::revLimit;

	if (scale > 1)
	{
		port /= scale;
		stbd /= scale;

		shared.tau.disable_axis.x = 1;
		shared.tau.disable_axis.z = 1;
	}

	shared.tau.wrench.force.x = port+stbd;
	shared.tau.wrench.torque.z = port-stbd;

	//Publish achieved tau
	shared.tauAch.publish(shared.tau);

	shared.thrusters[VRComms::port] = coerce(int(std::ceil(port)),
			-SharedData::revLimit, SharedData::revLimit);
	shared.thrusters[VRComms::stbd] = coerce(int(std::ceil(stbd)),
			-SharedData::revLimit, SharedData::revLimit);
	shared.thrusters[VRComms::light] = 0;

	shared.comms.encode(shared.thrusters);
	boost::asio::write(shared.port, boost::asio::buffer(shared.comms.outputBuffer));
	boost::asio::read(shared.port, boost::asio::buffer(shared.comms.inputBuffer));
	labust::vehicles::stateMapPtr states(new labust::vehicles::stateMap());
	shared.comms.decode(states);

	sensor_msgs::FluidPressure pressure;
	pressure.header.frame_id = "local";

	pressure.fluid_pressure = ((*states)[labust::vehicles::state::depthPressure]);

	sensor_msgs::Imu imu;
	Eigen::Quaternion<float> quat;
	labust::tools::quaternionFromEulerZYX(0.0, 0.0,
			((*states)[labust::vehicles::state::yaw]), quat);
	imu.orientation.x = quat.x();
	imu.orientation.y = quat.y();
	imu.orientation.z = quat.z();
	imu.orientation.w = quat.w();
	shared.imu.publish(imu);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"vr_node");
	ros::NodeHandle nh,ph("~");

	SharedData shared;
	ros::Subscriber tauIn = nh.subscribe<auv_msgs::BodyForceReq>("tauIn",1,boost::bind(&onTau,boost::ref(shared),_1));
	shared.tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);
	shared.imu = nh.advertise<sensor_msgs::Imu>("vr_imu",1);
	shared.depth = nh.advertise<sensor_msgs::FluidPressure>("vr_depth",1);

	//Get thruster configuration
	ph.param("Tnn", shared.Tnn,1.0);
	ph.param("_Tnn", shared._Tnn,1.0);

	//Get port name from configuration and open.
	std::string portName("/dev/ttyUSB0");
	int baud(9600);
	ph.param("PortName", portName,portName);
	ph.param("BaudRate", baud, baud);
	shared.portOpen(portName, baud);

	ros::spin();
	return 0;
}




