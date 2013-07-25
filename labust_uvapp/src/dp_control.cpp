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
#include <labust/control/DPControl.hpp>
#include <labust/control/HLControl.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>

#include <geometry_msgs/PointStamped.h>

namespace labust{namespace control{
///The fully actuated dynamic positioning controller
struct FADPControl
{
	enum {x=0,y,psi};

	FADPControl():Ts(0.1){};

	void init()
	{
		ros::NodeHandle nh;
		refTrack = nh.subscribe<auv_msgs::NavSts>("TrackPoint", 1,
				&FADPControl::onTrackPoint,this);
		refPoint = nh.subscribe<geometry_msgs::PointStamped>("LFPoint", 1,
				&FADPControl::onNewPoint,this);
		headingRef = nh.subscribe<std_msgs::Float32>("heading_ref", 1,
					&FADPControl::onHeadingRef,this);

		initialize_controller();
	}

	void onTrackPoint(const auv_msgs::NavSts::ConstPtr& ref)
	{
		boost::mutex::scoped_lock l(cnt_mux);
		trackPoint = *ref;

		con[x].desired =  trackPoint.position.north;
		con[y].desired =  trackPoint.position.east;
		con[x].feedforward=  trackPoint.body_velocity.x*cos(trackPoint.orientation.yaw);
		con[y].feedforward =  trackPoint.body_velocity.x*sin(trackPoint.orientation.yaw);
	};

	void onHeadingRef(const std_msgs::Float32::ConstPtr& hdg)
	{
		boost::mutex::scoped_lock l(cnt_mux);
		con[psi].desired = hdg->data;
	};

	void onNewPoint(const geometry_msgs::PointStamped::ConstPtr& point)
	{
		boost::mutex::scoped_lock l(cnt_mux);
		con[x].desired = point->point.x;
		con[y].desired = point->point.y;
	};

	void windup(const auv_msgs::BodyForceReq& tauAch)
	{
		//Copy into controller
		boost::mutex::scoped_lock l(cnt_mux);
		con[x].windup = tauAch.disable_axis.x;
		con[y].windup = tauAch.disable_axis.y;
		con[psi].windup = tauAch.disable_axis.yaw;
	};

	void step(const auv_msgs::NavSts::ConstPtr& state, auv_msgs::BodyVelocityReqPtr nu)
	{
		boost::mutex::scoped_lock l(cnt_mux);
		con[x].state = state->position.north;
		con[y].state = state->position.east;
		con[psi].state = state->orientation.yaw;

		PIFFExtController_step(&con[x],Ts);
		PIFFExtController_step(&con[y],Ts);
		PIFFExtController_step(&con[psi],Ts);

		nu->header.stamp = ros::Time::now();
		nu->goal.requester = "fadp_controller";

		Eigen::Vector2f out, in;
		Eigen::Matrix2f R;
		in<<con[x].output,con[y].output;
		double yaw(state->orientation.yaw);
		R<<cos(yaw),-sin(yaw),-in(yaw),cos(yaw);

		out = R.transpose()*in;

		nu->twist.linear.x = out[0];
		nu->twist.linear.y = out[1];
		nu->twist.angular.z = con[psi].output;
	}

	void initialize_controller()
	{
		ROS_INFO("Initializing dynamic positioning controller...");

		ros::NodeHandle nh;
		Eigen::Vector3d closedLoopFreq(Eigen::Vector3d::Ones());
		labust::tools::getMatrixParam(nh,"dp_controller/closed_loop_freq", closedLoopFreq);
		nh.param("dp_controller/sampling",Ts,Ts);

		enum {Kp=0, Ki, Kd, Kt};
		for (size_t i=0; i<3;++i)
		{
			PIDController_init(&con[i]);
			con[i].gains[Kp] = 2*closedLoopFreq(i);
			con[i].gains[Ki] = closedLoopFreq(i)*closedLoopFreq(i);
			con[i].autoTracking = 0;
		}

		ROS_INFO("Dynamic positioning controller initialized.");
	}

private:
	PIDController con[3];
	ros::Subscriber refTrack, refPoint, headingRef;
	auv_msgs::NavSts trackPoint;
	boost::mutex cnt_mux;
	double Ts;
};
}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"dp_control");

	bool underactuated(true);

	ros::NodeHandle nh;
	nh.param("dp_controller/underactuated",underactuated,underactuated);

	if (underactuated)
	{
		//Initialize
		labust::control::DPControl controller;
		//Start execution.
		controller.start();
	}
	else
	{
		labust::control::HLControl<labust::control::FADPControl,
			labust::control::EnablePolicy,
			labust::control::WindupPolicy> controller;
		ros::spin();
	}

	return 0;
}



