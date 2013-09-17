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
#include <labust/control/HeadingControl.hpp>
#include <labust/tools/MatrixLoader.hpp>

#include <auv_msgs/BodyVelocityReq.h>
#include <kdl/frames.hpp>
#include <boost/bind.hpp>

#include <cmath>
#include <vector>
#include <string>

using labust::control::HeadingControl;

HeadingControl::HeadingControl():
		nh(ros::NodeHandle()),
		ph(ros::NodeHandle("~")),
		lastEst(ros::Time::now()),
		timeout(0.5),
		Ts(0.1),
		safetyRadius(0.5),
		surge(1),
		flowSurgeEstimate(0),
		K1(0.2),
		K2(1),
		gammaARad(45),
		enable(false)
{this->onInit();}

void HeadingControl::onInit()
{
	//Initialize publishers
	nuRef = nh.advertise<auv_msgs::BodyVelocityReq>("nuRef", 1);
	vtTwist = nh.advertise<geometry_msgs::TwistStamped>("virtual_target_twist", 1);

	//Initialze subscribers
	stateHat = nh.subscribe<auv_msgs::NavSts>("stateHat", 1,
			&HeadingControl::onEstimate,this);
//	flowTwist = nh.subscribe<geometry_msgs::TwistStamped>("body_flow_frame_twist", 1,
//			&HeadingControl::onFlowTwist,this);
	headingRef = nh.subscribe<std_msgs::Float32>("heading_ref", 1,
				&HeadingControl::onHeadingRef,this);
	//	refTrack = nh.subscribe<auv_msgs::NavSts>("TrackPoint", 1,
	//			&HeadingControl::onTrackPoint,this);
	windup = nh.subscribe<auv_msgs::BodyForceReq>("tauAch", 1,
			&HeadingControl::onWindup,this);
	openLoopSurge = nh.subscribe<std_msgs::Float32>("open_loop_surge", 1,
			&HeadingControl::onOpenLoopSurge,this);
	enableControl = nh.advertiseService("HDG_enable",
			&HeadingControl::onEnableControl, this);

	nh.param("dp_controller/timeout",timeout,timeout);

	//Configure the dynamic reconfigure server
	//server.setCallback(boost::bind(&VelocityControl::dynrec_cb, this, _1, _2));

	initialize_controller();
	//config.__fromServer__(ph);
	//server.setConfigDefault(config);
	//this->updateDynRecConfig();
}

//void HeadingControl::updateDynRecConfig()
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

//void HeadingControl::dynrec_cb(labust_uvapp::VelConConfig& config, uint32_t level)
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

void HeadingControl::onHeadingRef(const std_msgs::Float32::ConstPtr& ref)
{
	headingController.desired = ref->data;
};

bool HeadingControl::onEnableControl(labust_uvapp::EnableControl::Request& req,
		labust_uvapp::EnableControl::Response& resp)
{
	this->enable = req.enable;
	return true;
}

void HeadingControl::onOpenLoopSurge(const std_msgs::Float32::ConstPtr& surge)
{
	this->surge = surge->data;
}

//void HeadingControl::onFlowTwist(const geometry_msgs::TwistStamped::ConstPtr& flowtwist)
//{
//	boost::mutex::scoped_lock l(dataMux);
//	flowSurgeEstimate = flowtwist->twist.linear.x*flowtwist->twist.linear.x;
//	flowSurgeEstimate += flowtwist->twist.linear.y*flowtwist->twist.linear.y;
//	flowSurgeEstimate = sqrt(flowSurgeEstimate);
//}

void HeadingControl::onEstimate(const auv_msgs::NavSts::ConstPtr& estimate)
{
	//Copy into controller
	state = *estimate;
	lastEst = ros::Time::now();
	if (enable) this->step();
};

//void HeadingControl::onTrackPoint(const auv_msgs::NavSts::ConstPtr& ref)
//{
//	//Copy into controller
//	trackPoint = *ref;
//};

void HeadingControl::onWindup(const auv_msgs::BodyForceReq::ConstPtr& tauAch)
{
	//Copy into controller
	headingController.windup = tauAch->disable_axis.yaw;
};

//void HeadingControl::safetyTest()
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

void HeadingControl::step()
{
	if (!enable) return;
	//this->safetyTest();

	headingController.state = labust::math::wrapRad(state.orientation.yaw);
	float errorWrap = labust::math::wrapRad(headingController.desired - headingController.state);
	PIFFExtController_stepWrap(&headingController,Ts, errorWrap);

	auv_msgs::BodyVelocityReq nu;
	nu.header.stamp = ros::Time::now();
	nu.goal.requester = "heading_controller";
	nu.twist.angular.z = headingController.output;
	nu.twist.linear.x = surge;

	nuRef.publish(nu);
}

void HeadingControl::start()
{
	ros::spin();
}

void HeadingControl::initialize_controller()
{
	ROS_INFO("Initializing heading controller...");

	double w(1);
	nh.param("heading_control/heading_closed_loop_freq", w,w);
	nh.param("heading_control/sampling",Ts,Ts);
	nh.param("heading_control/openLoopSurge",surge,surge);

	enum {Kp=0, Ki, Kd, Kt};
	PIDController_init(&headingController);
	headingController.gains[Kp] = 2*w;
	headingController.gains[Ki] = w*w;
	headingController.autoTracking = 0;

	ROS_INFO("Heading controller initialized.");
}
