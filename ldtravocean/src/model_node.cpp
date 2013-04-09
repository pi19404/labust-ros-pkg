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
 *  Created: 05.03.2013.
 *********************************************************************/
#include <labust/xml/XMLReader.hpp>
#include <labust/simulation/VehicleModel6DOF.hpp>
#include <labust/vehicles/ScaleAllocation.hpp>

#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyForceReq.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <gps_common/conversions.h>

#include <Eigen/Dense>

#include <boost/bind.hpp>

#include <sstream>
#include <string>

nav_msgs::Odometry* mapToUWSimOdometry(const labust::simulation::vector& eta,
		const labust::simulation::vector& nu,
		nav_msgs::Odometry* odom,
		tf::TransformListener& lisWorld)
{
	using namespace labust::simulation;
	using namespace Eigen;

	/*odom->pose.pose.position.x = eta(VehicleModel6DOF::x);
	odom->pose.pose.position.y = eta(VehicleModel6DOF::y);
	odom->pose.pose.position.z = eta(VehicleModel6DOF::z);
	Matrix3f m;
	m = AngleAxisf(eta(VehicleModel6DOF::psi), Vector3f::UnitZ())*
		AngleAxisf(eta(VehicleModel6DOF::theta), Vector3f::UnitY())*
		AngleAxisf(eta(VehicleModel6DOF::phi), Vector3f::UnitX());*/

	tf::StampedTransform transform;
	try
	{
	    lisWorld.lookupTransform("uwsim_frame", "base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException& ex)
	{
	   ROS_ERROR("%s",ex.what());
	}

	/*Quaternion<float> q(m);

	odom->pose.pose.orientation.x = q.x();
	odom->pose.pose.orientation.y = q.y();
	odom->pose.pose.orientation.z = q.z();
	odom->pose.pose.orientation.w = q.w();*/

  odom->twist.twist.linear.x = nu(VehicleModel6DOF::u);
	odom->twist.twist.linear.y = nu(VehicleModel6DOF::v);
	odom->twist.twist.linear.z = nu(VehicleModel6DOF::w);

	odom->twist.twist.angular.x = nu(VehicleModel6DOF::p);
	odom->twist.twist.angular.y = nu(VehicleModel6DOF::q);
	odom->twist.twist.angular.z = nu(VehicleModel6DOF::r);
	odom->child_frame_id = "base_link";

	odom->pose.pose.orientation.x = transform.getRotation().x();
	odom->pose.pose.orientation.y = transform.getRotation().y();
	odom->pose.pose.orientation.z = transform.getRotation().z();
	odom->pose.pose.orientation.w = transform.getRotation().w();

	odom->pose.pose.position.x = transform.getOrigin().x();
	odom->pose.pose.position.y = transform.getOrigin().y();
	odom->pose.pose.position.z = transform.getOrigin().z();

	odom->header.stamp = ros::Time::now();
	odom->header.frame_id = "uwsim_hook";

	return odom;
}

auv_msgs::NavSts* mapToNavSts(const labust::simulation::vector& eta, const labust::simulation::vector& nu, auv_msgs::NavSts* nav)
{
	using namespace labust::simulation;
	using namespace Eigen;
	nav->global_position.latitude = 0;
	nav->global_position.longitude = 0;

	nav->position.north = eta(VehicleModel6DOF::x);
	nav->position.east = eta(VehicleModel6DOF::y);
	nav->position.depth = eta(VehicleModel6DOF::z);
	nav->orientation.roll = eta(VehicleModel6DOF::phi);
	nav->orientation.pitch = eta(VehicleModel6DOF::theta);
	nav->orientation.yaw = eta(VehicleModel6DOF::psi);

	nav->body_velocity.x = nu(VehicleModel6DOF::u);
	nav->body_velocity.y = nu(VehicleModel6DOF::v);
	nav->body_velocity.z = nu(VehicleModel6DOF::w);
	nav->orientation_rate.roll = nu(VehicleModel6DOF::p);
	nav->orientation_rate.pitch = nu(VehicleModel6DOF::q);
	nav->orientation_rate.yaw = nu(VehicleModel6DOF::r);

	nav->header.stamp = ros::Time::now();

	return nav;
}

sensor_msgs::NavSatFix* mapToNavSatFix(const labust::simulation::vector& eta, const labust::simulation::vector& nu, sensor_msgs::NavSatFix* fix,
		const std::string& utmzone, tf::TransformListener& lisWorld, tf::TransformBroadcaster& gpsBroadcast)
{
	using namespace labust::simulation;
	using namespace Eigen;

	tf::StampedTransform transform;
	try
	{
	    lisWorld.lookupTransform("world", "gps_frame", ros::Time(0), transform);
	}
	catch (tf::TransformException& ex)
	{
	   ROS_ERROR("%s",ex.what());
	}

	fix->altitude = transform.getOrigin().z();
	gps_common::UTMtoLL(transform.getOrigin().y(), transform.getOrigin().x(), utmzone, fix->latitude, fix->longitude);
  fix->header.stamp = ros::Time::now();
  fix->header.frame_id = "world";

	return fix;
}

sensor_msgs::Imu* mapToImu(const labust::simulation::vector& eta,
		const labust::simulation::vector& nu, const labust::simulation::vector& nuacc,
		sensor_msgs::Imu* imu, tf::TransformBroadcaster& imuBroadcast)
{
	using namespace labust::simulation;
	using namespace Eigen;

	imu->header.stamp = ros::Time::now();
	imu->header.frame_id = "imu_frame";
	imu->linear_acceleration.x = nuacc(VehicleModel6DOF::x);
	imu->linear_acceleration.y = nuacc(VehicleModel6DOF::y);
	imu->linear_acceleration.z = nuacc(VehicleModel6DOF::z);
	imu->angular_velocity.x = nu(VehicleModel6DOF::p);
	imu->angular_velocity.y = nu(VehicleModel6DOF::q);
	imu->angular_velocity.z = nu(VehicleModel6DOF::r);
	tf::Quaternion quat = tf::createQuaternionFromRPY(eta(VehicleModel6DOF::phi),
			eta(VehicleModel6DOF::theta),eta(VehicleModel6DOF::psi));
	imu->orientation.x = quat.x();
	imu->orientation.y = quat.y();
	imu->orientation.z = quat.z();
	imu->orientation.w = quat.w();

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0, 0, 0));
	transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
	imuBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu_frame"));

	return imu;
}

void handleTau(labust::simulation::vector* tauIn, const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	using namespace labust::simulation;
	(*tauIn)(VehicleModel6DOF::X) = tau->wrench.force.x;
	(*tauIn)(VehicleModel6DOF::Y) = tau->wrench.force.y;
	(*tauIn)(VehicleModel6DOF::Z) = tau->wrench.force.z;
	(*tauIn)(VehicleModel6DOF::K) = tau->wrench.torque.x;
	(*tauIn)(VehicleModel6DOF::M) = tau->wrench.torque.y;
	(*tauIn)(VehicleModel6DOF::N) = tau->wrench.torque.z;
};

void handleCurrent(labust::simulation::vector* current, const std_msgs::String::ConstPtr& data)
{
	std::istringstream out(data->data);
	float a;
	out>>(*current)(0)>>(*current)(1)>>(*current)(2);;
};

//Simple dynamics simulation only ROS node
//\todo Add allocation algorithm
//\todo Add thruster nonlinearity ?
//\todo Separate GPS, Imu sim into simulation nodelets with noise
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"model_node");

	//Initialize the simulator
	labust::xml::ReaderPtr reader(new labust::xml::Reader(argv[1],true));
	reader->useNode(reader->value<_xmlNode*>("//configurations"));
	labust::simulation::VehicleModel6DOF model(reader);

	ros::NodeHandle nh,ph("~");

	//Publishers
	ros::Publisher state = nh.advertise<auv_msgs::NavSts>("meas",1);
	ros::Publisher stateNoisy = nh.advertise<auv_msgs::NavSts>("noisy_meas",1);
	ros::Publisher uwsim = nh.advertise<nav_msgs::Odometry>("uwsim_hook",1);
	ros::Publisher tauAch = nh.advertise<auv_msgs::BodyForceReq>("tauAch",1);
	ros::Publisher gpsFix = nh.advertise<sensor_msgs::NavSatFix>("fix",1);
	ros::Publisher imuMeas = nh.advertise<sensor_msgs::Imu>("imu_model",1);
	//Subscribers
	labust::simulation::vector tau(labust::simulation::zero_v(6)), current(labust::simulation::zero_v(3));
	ros::Subscriber tauSub = nh.subscribe<auv_msgs::BodyForceReq>("tauIn", 1, boost::bind(&handleTau,&tau,_1));
	ros::Subscriber curSub = nh.subscribe<std_msgs::String>("currents", 1, boost::bind(&handleCurrent,&current,_1));
	//Transform broadcasters
	tf::TransformBroadcaster localFrame;
	tf::TransformListener lisWorld;
	double originLat(0), originLon(0);
	nh.param("LocalOriginLat",originLat,originLat);
	nh.param("LocalOriginLon",originLon,originLon);

	double northing, easting;
	std::string utmzone;
	tf::Transform transform;
	gps_common::LLtoUTM(originLat, originLon,northing,easting,utmzone);
	std::cout.precision(10);
	std::cout<<"Northing:"<<northing<<","<<easting<<std::endl;
	transform.setOrigin(tf::Vector3(easting, northing, 0));
	transform.setRotation(tf::createQuaternionFromRPY(M_PI,0,M_PI/2));
	localFrame.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "local"));
	transform.setOrigin(tf::Vector3(originLon, originLat, 0));
	localFrame.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "localDeg"));

	auv_msgs::NavSts nav,navNoisy;
	nav_msgs::Odometry odom;
	sensor_msgs::NavSatFix fix;
	sensor_msgs::Imu imu;

	double fs(10);
	int wrap(1);
	ph.param("Rate",fs,fs);
	ph.param("ModelWrap",wrap,wrap);
	ros::Rate rate(fs);
	model.setTs(1/(fs*wrap));

	//Construct the allocation matrix for the vehicle.
	//We can have it fixed here since this is a specific application.
	//\todo Use real thruster angles instead of 45 degree
	Eigen::Matrix<float, 3,4> B;
	float cp(cos(M_PI/4)),sp(sin(M_PI/4));
	B<<cp,cp,cp,cp,
	   sp,-sp,-sp,sp,
	   1,-1,1,-1;

	double maxThrust(100/(2*cp)),minThrust(-maxThrust);
	ph.param("maxThrust",maxThrust,maxThrust);
	ph.param("minThrust",minThrust,-maxThrust);

	//Scaling allocation only for XYN
	labust::vehicles::ScaleAllocation allocator(B,maxThrust,minThrust);

	ros::Time lastGps = ros::Time::now();
	while (ros::ok())
	{
		using namespace labust::simulation;
		Eigen::Vector3f tauXYN,tauXYNsc;
		tauXYN<<tau(VehicleModel6DOF::X),tau(VehicleModel6DOF::Y),tau(VehicleModel6DOF::N);
		double scale = allocator.scale(tauXYN,&tauXYNsc);

		auv_msgs::BodyForceReq t;
		tau(VehicleModel6DOF::X) = t.wrench.force.x = tauXYNsc(0);
		tau(VehicleModel6DOF::Y) = t.wrench.force.y = tauXYNsc(1);
		t.wrench.force.z = tau(VehicleModel6DOF::Z);
		t.wrench.torque.x = tau(VehicleModel6DOF::K);
		t.wrench.torque.y = tau(VehicleModel6DOF::M);
		tau(VehicleModel6DOF::N) = t.wrench.torque.z = tauXYNsc(2);
		t.header.stamp = ros::Time::now();

		//Publish the scaled values if scaling occured
		if (scale>1)
		{
			//Signal windup occured
			t.disable_axis.x = t.disable_axis.y = t.disable_axis.yaw = 1;
		}

		tauAch.publish(t);

		model.setCurrent(current);
		//Perform simulation with smaller sampling type if wrap>1
		for (size_t i=0; i<wrap;++i) model.step(tau);

		uwsim.publish(*mapToUWSimOdometry(model.Eta(),model.Nu(),&odom, lisWorld));
		state.publish(*mapToNavSts(model.Eta(),model.Nu(),&nav));
		stateNoisy.publish(*mapToNavSts(model.EtaNoisy(),model.NuNoisy(),&navNoisy));

		if ((ros::Time::now()-lastGps).sec >=1)
		{
			mapToNavSatFix(model.Eta(),model.Nu(),&fix,utmzone,lisWorld,localFrame);
			if (fix.altitude >= 0)
			{
				gpsFix.publish(fix);
			}
				lastGps = ros::Time::now();
		}
		imuMeas.publish(*mapToImu(model.Eta(),model.Nu(),model.NuAcc(),&imu,localFrame));

		transform.setOrigin(tf::Vector3(easting, northing, 0));
		transform.setRotation(tf::createQuaternionFromRPY(M_PI,0,M_PI/2));
		localFrame.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "local"));

		const vector& eta = model.Eta();
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(eta(VehicleModel6DOF::x),
				eta(VehicleModel6DOF::y),
				eta(VehicleModel6DOF::z)));
		transform.setRotation(tf::createQuaternionFromRPY(eta(VehicleModel6DOF::phi),eta(VehicleModel6DOF::theta),eta(VehicleModel6DOF::psi)));
		localFrame.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local", "base_link"));

		tf::Transform transform2;
		transform2.setOrigin(tf::Vector3(0, 0, -0.8));
		transform2.setRotation(tf::createQuaternionFromRPY(0,0,0));
		localFrame.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "gps_frame"));

		tf::Transform transform3;
		transform3.setOrigin(tf::Vector3(0, 0, 0));
		transform3.setRotation(tf::createQuaternionFromRPY(M_PI,0,M_PI/2));
		localFrame.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "local", "uwsim_frame"));

		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

