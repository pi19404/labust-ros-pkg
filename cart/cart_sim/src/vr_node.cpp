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
#include <labust/vehicles/UVApp.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/navigation/KFCore.hpp>
#include <labust/navigation/KinematicModel.hpp>

#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/VehiclePose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <Eigen/Dense>

#include <boost/bind.hpp>

#include <iostream>


void mapToOdometry(labust::vehicles::stateMap& stateHat, nav_msgs::Odometry* odom)
{
	using namespace labust::vehicles;
	using namespace Eigen;
	odom->pose.pose.position.x = stateHat[state::x];
	odom->pose.pose.position.y = stateHat[state::y];
	odom->pose.pose.position.z = stateHat[state::z];
	Matrix3f m;
	m = AngleAxisf(stateHat[state::roll], Vector3f::UnitX())
	* AngleAxisf(stateHat[state::pitch], Vector3f::UnitY())
	* AngleAxisf(stateHat[state::yaw], Vector3f::UnitZ());
	Quaternion<float> q(m);

	odom->pose.pose.orientation.x = q.x();
	odom->pose.pose.orientation.y = q.y();
	odom->pose.pose.orientation.z = q.z();
	odom->pose.pose.orientation.w = q.w();

	odom->twist.twist.linear.x = stateHat[state::u];
	odom->twist.twist.linear.y = stateHat[state::v];
	odom->twist.twist.linear.z = stateHat[state::w];

	odom->twist.twist.angular.x = stateHat[state::p];
	odom->twist.twist.angular.y = stateHat[state::q];
	odom->twist.twist.angular.z = stateHat[state::r];
}

void mapToNavSts(labust::vehicles::stateMap& stateHat, auv_msgs::NavSts* nav)
{
	using namespace labust::vehicles;
	using namespace Eigen;
	nav->global_position.latitude = stateHat[state::lat];
	nav->global_position.longitude = stateHat[state::lon];

	nav->position.north = stateHat[state::x];
	nav->position.east = stateHat[state::y];
	nav->position.depth = stateHat[state::z];
	nav->orientation.roll = stateHat[state::roll];
	nav->orientation.pitch = stateHat[state::pitch];
	nav->orientation.yaw = stateHat[state::yaw];

	nav->body_velocity.x = stateHat[state::u];
	nav->body_velocity.y = stateHat[state::v];
	nav->body_velocity.z = stateHat[state::w];
	nav->orientation_rate.roll = stateHat[state::p];
	nav->orientation_rate.pitch = stateHat[state::q];
	nav->orientation_rate.yaw = stateHat[state::r];

	nav->header.stamp = ros::Time::now();
}

void handleTau(labust::vehicles::UVApp* app,const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	using namespace labust::vehicles;
	labust::vehicles::tauMap map;
	map[tau::X] = tau->wrench.force.x;
	map[tau::Y] = tau->wrench.force.y;
	map[tau::Z] = tau->wrench.force.z;
	map[tau::K] = 0;tau->wrench.torque.x;
	map[tau::M] = 0;tau->wrench.torque.y;
	map[tau::N] = tau->wrench.torque.z;

	app->setExternalTau(map);
};

void handleJoy(labust::vehicles::UVApp* app,const sensor_msgs::Joy::ConstPtr& joy)
{
	using namespace labust::vehicles;
	labust::vehicles::tauMap map;
	map[tau::X] = joy->axes[1];
	map[tau::Y] = joy->axes[0];
	map[tau::Z] = joy->axes[3];
	map[tau::K] = 0;
	map[tau::M] = 0;
	map[tau::N] = -joy->axes[2];

	app->setExternalTau(map);
};

void handleRef(labust::vehicles::stateMap* map,const auv_msgs::VehiclePose::ConstPtr& ref)
{
	using namespace labust::vehicles;
	(*map)[state::x] = ref->position.north;
	(*map)[state::y] = ref->position.east;
	(*map)[state::z] = ref->position.depth;
	(*map)[state::roll] = ref->orientation.roll;
	(*map)[state::pitch] = ref->orientation.pitch;
	(*map)[state::yaw] = ref->orientation.yaw;
};

void handleMode(labust::vehicles::UVApp* app, const std_msgs::Int32::ConstPtr& mode)
{
	ROS_INFO("Received mode: %d",labust::vehicles::UVApp::UVMode(mode->data));
	if (mode->data == 0) app->setUVMode(labust::vehicles::UVApp::idle);
	if (mode->data == 1) app->setUVMode(labust::vehicles::UVApp::manual);
	if (mode->data == 5) app->setUVMode(labust::vehicles::UVApp::dynamicPositioning);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"vr_node");

	labust::xml::ReaderPtr reader(new labust::xml::Reader(argv[1],true));
	reader->useNode(reader->value<_xmlNode*>("//configurations"));
	labust::vehicles::UVApp app(reader,"vr");

	ros::NodeHandle nh;

	//Publishers
	ros::Publisher pub_state = nh.advertise<auv_msgs::NavSts>("stateHat",1);
	ros::Publisher pub_meas = nh.advertise<auv_msgs::NavSts>("meas",1);
	ros::Publisher uwsim_hook = nh.advertise<nav_msgs::Odometry>("uwsim_hook",1);

	labust::vehicles::stateMap stateRef;
	//Subscribers
	ros::Subscriber tauIn = nh.subscribe<auv_msgs::BodyForceReq>("tauIn", 1, boost::bind(&handleTau,&app,_1));
	ros::Subscriber joyIn = nh.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&handleJoy,&app,_1));
	ros::Subscriber refIn = nh.subscribe<auv_msgs::VehiclePose>("refIn", 1, boost::bind(&handleRef,&stateRef,_1));
	ros::Subscriber modeIn = nh.subscribe<std_msgs::Int32>("modeIn", 1, boost::bind(&handleMode,&app,_1));

	ros::Rate rate(10);

	while (ros::ok())
	{
		using namespace labust::vehicles;
		labust::vehicles::stateMap stateHat, ext_measurements;
		tauMap tau;

		/*app.setExternalRef(stateRef);
		app.setUVMode(UVApp::manual);
		tau[tau::X] = 1.5;
		tau[tau::Y] = -1.5;*/
		//app.setExternalTau(tau);
		stateHat = app.step(ext_measurements);

		std::cout<<"References:"<<stateRef[state::z]<<std::endl;

		//Estimated states
		auv_msgs::NavSts nav;
		mapToNavSts(ext_measurements, &nav);
		pub_state.publish(nav);

		//Estimated states
		auv_msgs::NavSts meas;
		mapToNavSts(ext_measurements, &meas);
		//pub_meas.publish(meas);

		//UWSim odometry message hook
		nav_msgs::Odometry odom;
		mapToOdometry(ext_measurements, &odom);
		uwsim_hook.publish(odom);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

