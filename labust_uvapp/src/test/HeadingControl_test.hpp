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
#ifndef HEADINGCONTROL_TEST_HPP_
#define HEADINGCONTROL_TEST_HPP_
#include <labust/control/HLControl.hpp>
#include <labust/control/PIDController.h>
#include <labust/math/NumberManipulation.hpp>
#include <std_msgs/Float32.h>

struct HeadingControl
{
	HeadingControl():Ts(0.1){};

	void init()
	{
		ros::NodeHandle nh;
		headingRef = nh.subscribe<std_msgs::Float32>("heading_ref", 1,
					&HeadingControl::onHeadingRef,this);

		initialize_controller();
	}

	void onHeadingRef(const std_msgs::Float32::ConstPtr& ref)
	{
		headingController.desired = ref->data;
	};

	void windup(const auv_msgs::BodyForceReq& tauAch)
	{
		//Copy into controller
		headingController.windup = tauAch.disable_axis.yaw;
	};

	void step(const auv_msgs::NavSts::ConstPtr& state, auv_msgs::BodyVelocityReqPtr nu)
	{
		//if (!enable) return;
		//this->safetyTest();
		headingController.state = labust::math::wrapRad(state->orientation.yaw);
		float errorWrap = labust::math::wrapRad(headingController.desired - headingController.state);
		PIFFExtController_stepWrap(&headingController,Ts, errorWrap);

		nu->header.stamp = ros::Time::now();
		nu->goal.requester = "heading_controller";
		nu->twist.angular.z = headingController.output;
	}

	void initialize_controller()
	{
		ROS_INFO("Initializing heading controller...");

		double w(1);
		ros::NodeHandle nh;
		nh.param("heading_control/heading_closed_loop_freq", w,w);
		nh.param("heading_control/sampling",Ts,Ts);

		enum {Kp=0, Ki, Kd, Kt};
		PIDController_init(&headingController);
		headingController.gains[Kp] = 2*w;
		headingController.gains[Ki] = w*w;
		headingController.autoTracking = 0;

		ROS_INFO("Heading controller initialized.");
	}

private:
	PIDController headingController;
	ros::Subscriber headingRef;
	double Ts;
};

#endif



