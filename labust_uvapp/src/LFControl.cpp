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
 *  Created: 03.05.2013.
 *********************************************************************/
#include <labust/control/LFControl.hpp>
#include <labust/tools/rosutils.hpp>

#include <auv_msgs/BodyVelocityReq.h>
#include <boost/bind.hpp>

#include <cmath>
#include <vector>
#include <string>

using labust::control::LFControl;

LFControl::LFControl():
			nh(ros::NodeHandle()),
			ph(ros::NodeHandle("~")),
			lastEst(ros::Time::now()),
			timeout(0.5),
			Ts(0.1),
			surge(0.5),
			currSurge(0.5),
			T0(labust::navigation::LFModel::zeros(3)),
			enable(false)
{this->onInit();}

void LFControl::onInit()
{;
	//Initialize publishers
	nuRef = nh.advertise<auv_msgs::BodyVelocityReq>("nuRef", 1);

	//Initialze subscribers
	stateHat = nh.subscribe<auv_msgs::NavSts>("stateHat", 1,
			&LFControl::onEstimate,this);
	refPoint = nh.subscribe<geometry_msgs::PointStamped>("LFPoint", 1,
			&LFControl::onNewPoint,this);
	enableControl = nh.advertiseService("LF_enable",
			&LFControl::onEnableControl, this);

	nh.param("lf_controller/timeout",timeout,timeout);

	//Configure the dynamic reconfigure server
	//server.setCallback(boost::bind(&VelocityControl::dynrec_cb, this, _1, _2));

	initialize_controller();
	//config.__fromServer__(ph);
	//server.setConfigDefault(config);
	//this->updateDynRecConfig();
}

//void LFControl::updateDynRecConfig()
//{
//	ROS_INFO("Updating the dynamic reconfigure parameters.");
//
//	config.__fromServer__(ph);
//	config.Surge_mode = axis_control[u];
//	config.Sway_mode = axis_control[v];
//	config.Heave_mode = axis_control[w];
//	config.Roll_mode = axis_control[p];
//	config.Pitch_mode = axis_control[q];
//	config.Yaw_mode = axis_control[r];
//
//	config.High_level_controller="0 - None\n 1 - DP";
//
//	server.updateConfig(config);
//}

//void LFControl::dynrec_cb(labust_uvapp::VelConConfig& config, uint32_t level)
//{
//	this->config = config;
//
//	for(size_t i=u; i<=r; ++i)
//	{
//		int newMode(0);
//		ph.getParam(dofName[i]+"_mode", newMode);
//		//Stop the identification if it was aborted remotely.
//		if ((axis_control[i] == identAxis) &&
//				(newMode != identAxis) &&
//				(ident[i] != 0)) ident[i].reset();
//
//		axis_control[i] = newMode;
//	}
//}

void LFControl::onNewPoint(const geometry_msgs::PointStamped::ConstPtr& point)
{
	using labust::navigation::LFModel;
	LFModel::vector Tt(labust::navigation::LFModel::zeros(3));
	Tt(0) = point->point.x;
	Tt(1) = point->point.y;
	Tt(2) = point->point.z;

	line.setLine(T0,Tt);
};

bool LFControl::onEnableControl(labust_uvapp::EnableControl::Request& req,
		labust_uvapp::EnableControl::Response& resp)
{
	this->enable = req.enable;
	return true;
}

void LFControl::onEstimate(const auv_msgs::NavSts::ConstPtr& estimate)
{
	//Copy into controller
	//Calculate DH.
	T0(0) = estimate->position.north;
	T0(1) = estimate->position.east;
	T0(2) = estimate->position.depth;

	currSurge = estimate->body_velocity.x;
	currYaw = estimate->orientation.yaw;

	lastEst = ros::Time::now();
	if (enable) this->step();
	//newEstimate = true;
	//if (newReference) step();
};

//void LFControl::safetyTest()
//{
//	bool refTimeout = (ros::Time::now() - lastRef).toSec() > timeout;
//	bool estTimeout = (ros::Time::now() - lastEst).toSec() > timeout;
//	bool manTimeout = (ros::Time::now() - lastMan).toSec() > timeout;
//	bool measTimeout = (ros::Time::now() - lastMeas).toSec() > timeout;
//	bool changed = false;
//
//	for (int i=u; i<=r;++i)
//	{
//		bool cntChannel = (refTimeout || estTimeout) && (axis_control[i] == controlAxis);
//		if (cntChannel) ROS_WARN("Timeout on the control channel. Controlled axes will be disabled.");
//		bool measChannel = measTimeout && (axis_control[i] == identAxis);
//		if (measChannel) ROS_WARN("Timeout on the measurement channel. Stopping identifications in progress.");
//		bool manChannel = manTimeout && (axis_control[i] == manualAxis);
//		if (manChannel) ROS_WARN("Timeout on the manual channel. Manual axes will be disabled.");
//
//		suspend_axis[i] = (cntChannel || measChannel || manChannel);
//	}
//
//	//Update only on change.
//	//if (changed) this->updateDynRecConfig();
//}

void LFControl::step()
{
	if (!enable) return;

	adjustDH();
	auv_msgs::BodyVelocityReq nu;
	//this->safetyTest();
	nu.goal.requester = "lf_control";
	nu.twist.linear.x = 0.5;
	if (fabs(currYaw - line.gamma()) < M_PI/2)
	{
		double dd = currSurge*sin(currYaw - line.gamma());
		nu.twist.angular.z = dh_controller.step(0,-line.calculatedH(T0(0),T0(1),T0(2)));
		std::cout<<nu.twist.angular.z<<", dH:"<<line.calculatedH(T0(0),T0(1),T0(2))<<std::endl;
	}
	else
	{
		nu.twist.angular.z = fabs(line.gamma() - currYaw);
		std::cout<<"Direct turn control"<<std::endl;
	}

	nuRef.publish(nu);
}

void LFControl::start()
{
	ros::spin();
}

void LFControl::adjustDH()
{
	//Check for zero surge
	if (fabs(currSurge) < 0.1) currSurge=0.1;

	double Kph = wh*wh/currSurge;
	double Kdh = 2*wh/currSurge;
	double aAngle = M_PI/4;
	this->dh_controller.setGains(Kph,0, Kdh,0);
	double dsat(Kdh/Kph*currSurge*sin(aAngle));

	labust::math::Limit<double> limit(-Kph*dsat,Kph*dsat);
	this->dh_controller.setPLimits(limit);
}

void LFControl::initialize_controller()
{
	ROS_INFO("Initializing line following controller...");

	nh.param("lf_controller/closed_loop_freq",wh,0.2);
	nh.param("lf_controller/default_surge",surge,surge);
	nh.param("lf_controller/sampling",Ts,Ts);
	dh_controller.setTs(Ts);
	adjustDH();

	ROS_INFO("Line following controller initialized.");
}
