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
#include <labust/control/DPControl.hpp>
#include <labust/tools/rosutils.hpp>

#include <auv_msgs/BodyVelocityReq.h>
#include <boost/bind.hpp>

#include <cmath>
#include <vector>
#include <string>

using labust::control::DPControl;

DPControl::DPControl():
			nh(ros::NodeHandle()),
			ph(ros::NodeHandle("~")),
			lastEst(ros::Time::now()),
			timeout(0.5),
			Ts(0.1),
			safetyRadius(0.5),
			surge(1),
			enable(false),
			inRegion(false)
{this->onInit();}

void DPControl::onInit()
{;
	//Initialize publishers
	nuRef = nh.advertise<auv_msgs::BodyVelocityReq>("nuRef", 1);

	//Initialze subscribers
	stateHat = nh.subscribe<auv_msgs::NavSts>("stateHat", 1,
			&DPControl::onEstimate,this);
	refPoint = nh.subscribe<geometry_msgs::PointStamped>("LFPoint", 1,
			&DPControl::onNewPoint,this);
	refTrack = nh.subscribe<auv_msgs::NavSts>("TrackPoint", 1,
			&DPControl::onTrackPoint,this);
	windup = nh.subscribe<auv_msgs::BodyForceReq>("tauAch", 1,
			&DPControl::onWindup,this);
	enableControl = nh.advertiseService("DP_enable",
			&DPControl::onEnableControl, this);

	nh.param("dp_controller/timeout",timeout,timeout);

	//Configure the dynamic reconfigure server
	//server.setCallback(boost::bind(&VelocityControl::dynrec_cb, this, _1, _2));

	initialize_controller();
	//config.__fromServer__(ph);
	//server.setConfigDefault(config);
	//this->updateDynRecConfig();
}

//void DPControl::updateDynRecConfig()
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

//void DPControl::dynrec_cb(labust_uvapp::VelConConfig& config, uint32_t level)
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

void DPControl::onNewPoint(const geometry_msgs::PointStamped::ConstPtr& point)
{
	trackPoint.position.north = point->point.x;
	trackPoint.position.east = point->point.y;
	trackPoint.position.depth = point->point.z;
};

bool DPControl::onEnableControl(labust_uvapp::EnableControl::Request& req,
		labust_uvapp::EnableControl::Response& resp)
{
	this->enable = req.enable;
	return true;
}

void DPControl::onEstimate(const auv_msgs::NavSts::ConstPtr& estimate)
{
	//Copy into controller
	state = *estimate;
	lastEst = ros::Time::now();
	if (enable) this->step();
};

void DPControl::onTrackPoint(const auv_msgs::NavSts::ConstPtr& ref)
{
	//Copy into controller
	trackPoint = *ref;
};

void DPControl::onWindup(const auv_msgs::BodyForceReq::ConstPtr& tauAch)
{
	//Copy into controller
	headingController.windup = tauAch->disable_axis.yaw;
	distanceController.windup = tauAch->disable_axis.x;
};

//void DPControl::safetyTest()
//{
//	bool refTimeout = (ros::Time::now() - lastRef).toSec() > timeout;
//	bool estTimeout = (ros::Time::now() - lastEst).toSec() > timeout;
//	bool manTimeout = (ros::Time::now() - lastMan).toSec() > timeout;
//	bool measTimeout = (ros::Time::now() - lastMeas).toSec() > timeout;
//	bool changed = false;
//
//	for (int i=u; i<=r;++i)
//	{
//		bool cntChannel = (refT*imeout || estTimeout) && (axis_control[i] == controlAxis);
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

void DPControl::step()
{
	if (!enable) return;
	//this->safetyTest();

	auv_msgs::BodyVelocityReq nu;
	//For 2D non-holonomic case
	double dy(trackPoint.position.east - state.position.east);
	double dx(trackPoint.position.north - state.position.north);
	//double angle = state.orientation.yaw;
	//dy += safetyRadius*sin(angle);
	//dx += safetyRadius*cos(angle);
	double dist(sqrt(dy*dy+dx*dx));
	double angleDiff(labust::math::wrapRad(fabs(atan2(dy,dx) - labust::math::wrapRad(state.orientation.yaw))));

	distanceController.desired = -0.5*safetyRadius;
	distanceController.state = -dist;
	headingController.state = state.orientation.yaw;

	headingController.desired = atan2(dy,dx);
	headingController.feedforward = trackPoint.orientation_rate.yaw;
	distanceController.feedforward = sqrt(pow(trackPoint.body_velocity.x,2) + pow(trackPoint.body_velocity.y,2));

	float errorWrap = labust::math::wrapRad(headingController.desired - headingController.state);
	PIFFExtController_stepWrap(&headingController,Ts, errorWrap);

	nu.goal.requester = "dp_control";
	nu.twist.angular.z = headingController.output;
	//Limit the outgoing surge to a sensible value
	nu.twist.linear.x = 0;
	if ((dist < 5*safetyRadius) && (angleDiff < M_PI/2))
	{
		//distanceController.state = -dx*cos(state.orientation.yaw) - dy*sin(state.orientation.yaw);
		PIFFExtController_step(&distanceController,Ts);
		nu.twist.linear.x = distanceController.output;
	}
	else if (angleDiff < M_PI/2)
	{
		nu.twist.linear.x = surge;
	}

	inRegion = (dist < safetyRadius) || (inRegion && (dist < 2*safetyRadius));

	if (inRegion)
	{
		nu.twist.linear.x = 0;
		nu.twist.angular.z = 0;
	}

	nuRef.publish(nu);
}

void DPControl::start()
{
	ros::spin();
}

void DPControl::initialize_controller()
{
	ROS_INFO("Initializing dynamic positioning controller...");

	Eigen::Vector2d closedLoopFreq(Eigen::Vector2d::Ones());
	labust::tools::getMatrixParam(nh,"dp_controller/closed_loop_freq", closedLoopFreq);
	nh.param("dp_controller/sampling",Ts,Ts);
	nh.param("dp_controller/safetyRadius",safetyRadius, safetyRadius);
	nh.param("dp_controller/openLoopSurge",surge, surge);

	enum {Kp=0, Ki, Kd, Kt};
	PIDController_init(&headingController);
	headingController.gains[Kp] = 2*closedLoopFreq(1);
	headingController.gains[Ki] = closedLoopFreq(1)*closedLoopFreq(1);
	headingController.autoTracking = 0;

	PIDController_init(&distanceController);
	distanceController.gains[Kp] = 2*closedLoopFreq(0);
	distanceController.gains[Ki] = closedLoopFreq(0)*closedLoopFreq(0);
	distanceController.autoTracking = 0;

	ROS_INFO("Line following controller initialized.");
}
