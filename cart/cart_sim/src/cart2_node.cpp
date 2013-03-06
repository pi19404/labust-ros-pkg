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
#include <underwater_sensor_msgs/USBL.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <Eigen/Dense>

#include <boost/bind.hpp>

#include <iostream>

typedef labust::navigation::KFCore<labust::navigation::KinematicModel> USBLFilter;

boost::mt19937 rng_; ///< Boost random number generator


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
	if (mode->data == 2) app->setUVMode(labust::vehicles::UVApp::headingDepth);
	if (mode->data == 5) app->setUVMode(labust::vehicles::UVApp::dynamicPositioning);
}

void handleUSBL(std::pair<bool,underwater_sensor_msgs::USBL>* msgOut, const underwater_sensor_msgs::USBL::ConstPtr& msgIn)
{
	(*msgOut).second = *msgIn;
	msgOut->first = true;
}

double xpp=5, ypp=2;

void handleUSBL2(std::pair<bool,underwater_sensor_msgs::USBL>* msgOut, const auv_msgs::NavSts::ConstPtr& msgIn)
{
	float std_ = 0.5;
	static boost::normal_distribution<> normal(0,std_);
	static boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng_, normal);
	static int it = 0;
	++it;
	msgOut->first = (it%10)==0;

	if (msgOut->first)
	{
//		double dx = msgIn->position.north - xpp;
//		double dy = msgIn->position.east - ypp;
//		double dz = 10;
//		double slant = sqrt(dx*dx + dy*dy + dz*dz) +  0.1*var_nor();
//		double bearing = atan2(dy, dx) + 5*M_PI/180*var_nor();
//		double elevation = atan2(dz, sqrt(dx*dx + dy*dy)) + 5*M_PI/180*var_nor();
//		(*msgOut).second.position.x = xpp + slant*cos(elevation)*cos(bearing);
//		(*msgOut).second.position.y = ypp + slant*cos(elevation)*sin(bearing);
//
//		std::cout<<"diff:"<<(*msgOut).second.position.x - msgIn->position.north<<std::endl;
//		std::cout<<"diff:"<<(*msgOut).second.position.y - msgIn->position.east<<std::endl;

		(*msgOut).second.position.x = msgIn->position.north + var_nor();
		(*msgOut).second.position.y = msgIn->position.east + var_nor();
	}
	//msgOut->first = true;
}

void handleOceanInfo(labust::vehicles::UVApp* app, const geometry_msgs::Quaternion::ConstPtr& msgIn)
{
	std::ostringstream str;
	str<<"<uvapp><UVSim><param name=\"waterLevel\" value=\""<<-msgIn->w<<"\" type=\"double\" /></UVSim></uvapp>";
	//app->setCommand(str.str());
}

Eigen::Vector2f stateDP;

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"cart2_node");

	labust::xml::ReaderPtr reader(new labust::xml::Reader(argv[1],true));
	reader->useNode(reader->value<_xmlNode*>("//configurations"));
	labust::vehicles::UVApp app(reader,"cart2");
	USBLFilter filter;
	filter.setTs(0.1);
	filter.setStateCovariance(10*USBLFilter::eye(USBLFilter::stateNum));

	ros::NodeHandle nh;

	//Publishers
	ros::Publisher pub_state = nh.advertise<auv_msgs::NavSts>("stateHat",1);
	ros::Publisher pub_meas = nh.advertise<auv_msgs::NavSts>("meas",1);
	ros::Publisher uwsim_hook = nh.advertise<nav_msgs::Odometry>("uwsim_hook",1);

	//ros::Publisher pub_full = nh.advertise<auv_msgs::BodyVelocityReq>("nuRef",10);
	ros::Publisher usbl_full = nh.advertise<auv_msgs::NavSts>("usblEstimate",1);
	ros::Publisher usbl_meas = nh.advertise<auv_msgs::NavSts>("usblMeas",1);

	std::pair<bool,underwater_sensor_msgs::USBL> msg;
	labust::vehicles::stateMap stateRef;

	//Subscribers
	ros::Subscriber tauIn = nh.subscribe<auv_msgs::BodyForceReq>("tauIn", 1, boost::bind(&handleTau,&app,_1));
	ros::Subscriber refIn = nh.subscribe<auv_msgs::VehiclePose>("refIn", 1, boost::bind(&handleRef,&stateRef,_1));
	ros::Subscriber modeIn = nh.subscribe<std_msgs::Int32>("modeIn", 1, boost::bind(&handleMode,&app,_1));
	//ros::Subscriber usblIn2 = nh.subscribe<underwater_sensor_msgs::USBL>("usbl", 10, boost::bind(&handleUSBL,&msg,_1));
	ros::Subscriber usblIn = nh.subscribe<auv_msgs::NavSts>("/diver/stateHat", 1, boost::bind(&handleUSBL2,&msg,_1));
	ros::Subscriber oceanInfo = nh.subscribe<geometry_msgs::Quaternion>("ocean_info", 1, boost::bind(&handleOceanInfo,&app,_1));

	ros::Rate rate(10);

	while (ros::ok())
	{
		using namespace labust::vehicles;
		labust::vehicles::stateMap stateHat, ext_measurements;
		tauMap tau;

		app.setExternalRef(stateRef);
		app.setUVMode(UVApp::manual);
		tau[tau::X] = 1.5;
		tau[tau::Y] = -1.5;
		//app.setExternalTau(tau);
		stateHat = app.step(ext_measurements);
		xpp = ext_measurements[state::x];
		ypp = ext_measurements[state::y];
		std::cout<<"Speed:"<<stateHat[state::u]<<std::endl;

		filter.predict();

		if (msg.first)
		{
			USBLFilter::input_type vec(2);
			vec(0) = msg.second.position.x;
			vec(1) = msg.second.position.y;
			filter.correct(vec);
		}

		USBLFilter::output_type data = filter.getState();

		stateRef[state::x] = data(USBLFilter::xp);
		stateRef[state::y] = data(USBLFilter::yp);

		auv_msgs::NavSts usbl_estimate;
		usbl_estimate.orientation.yaw = data(USBLFilter::psi);
		usbl_estimate.position.north = data(USBLFilter::xp);
		usbl_estimate.position.east = data(USBLFilter::yp);
		usbl_estimate.body_velocity.x = data(USBLFilter::Vv);
		usbl_estimate.orientation_rate.yaw = data(USBLFilter::r);
		usbl_estimate.header.stamp = ros::Time::now();

		usbl_full.publish(usbl_estimate);

		//if (msg.first)
		{
			auv_msgs::NavSts usblMeas;
			usblMeas.position.north = msg.second.position.x;;
			usblMeas.position.east = msg.second.position.y;
			usblMeas.header.stamp = ros::Time::now();
			usbl_meas.publish(usblMeas);
		}

		msg.first = false;

//		//Nikola's DP/////////////////////////////////////////////////
//		Eigen::Matrix2f R;
//		R(0,0) = R(1,1) = cos(stateHat[state::yaw]);
//		R(1,0) = -sin(stateHat[state::yaw]);
//		R(0,1) = -R(1,0);
//
//		Eigen::Matrix2f Kp;
//		Kp(0,0) = 0.3;
//		Kp(1,1) = 0.3;
//		Kp(0,1) = 0;
//		Kp(1,0) = 0;
//
//		Eigen::Vector2f d,ff,nuRef;
//		d(0) = ext_measurements[state::x] - data(USBLFilter::xp);
//		d(1) = ext_measurements[state::y] - data(USBLFilter::yp);
//
//		ff(0) = data(USBLFilter::Vv) * cos(data(USBLFilter::psi));
//		ff(1) = data(USBLFilter::Vv) * sin(data(USBLFilter::psi));
//
//		nuRef = R.transpose()*(-Kp*d + ff);
//
//		auv_msgs::BodyVelocityReq req;
//		req.twist.linear.x = nuRef(0);
//		req.twist.linear.y = nuRef(1);
//		req.twist.angular.z = 2*(stateRef[state::yaw] - ext_measurements[state::yaw]);
//
//		req.disable_axis.roll = req.disable_axis.pitch = req.disable_axis.z = true;
//
//		//pub_full.publish(req);
//		/////////////////////////////////////////////////////////////

		std::cout<<"References:"<<stateRef[state::z]<<std::endl;

		std::cout<<"Desired position:"<<data(USBLFilter::xp)<<std::endl;

		//Estimated states
		auv_msgs::NavSts nav;
		mapToNavSts(ext_measurements, &nav);
		pub_state.publish(nav);

		//Estimated states
		auv_msgs::NavSts meas;
		mapToNavSts(ext_measurements, &meas);
		pub_meas.publish(meas);

		//UWSim odometry message hook
		nav_msgs::Odometry odom;
		mapToOdometry(ext_measurements, &odom);
		uwsim_hook.publish(odom);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

