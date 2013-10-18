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
#include <labust/control/HLControl.hpp>
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/WindupPolicy.hpp>
#include <labust/control/PIFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The fully actuated dynamic positioning controller
		struct FADPControl
		{
			enum {x=0,y,psi};

			FADPControl():Ts(0.1), uff(0), vff(0){};

			void init()
			{
				ros::NodeHandle nh;
				headingRef = nh.subscribe<std_msgs::Float32>("heading_ref", 1,
						&FADPControl::onHeadingRef,this);

				initialize_controller();
			}

			void onHeadingRef(const std_msgs::Float32::ConstPtr& hdg)
			{
				con[psi].desired = hdg->data;
			};

			void windup(const auv_msgs::BodyForceReq& tauAch)
			{
				//Copy into controller
				con[x].windup = tauAch.disable_axis.x;
				con[y].windup = tauAch.disable_axis.y;
				con[psi].windup = tauAch.disable_axis.yaw;
			};

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state)
			{
				con[x].desired =  ref.position.north;
				con[y].desired =  ref.position.east;
				uff = ref.body_velocity.x*cos(ref.orientation.yaw);
				vff = ref.body_velocity.x*sin(ref.orientation.yaw);

				con[x].state = state.position.north;
				con[y].state = state.position.east;
				con[psi].state = state.orientation.yaw;

				PIFF_ffStep(&con[x],Ts, uff);
				PIFF_ffStep(&con[y],Ts, vff);
				float errorWrap = labust::math::wrapRad(
						con[psi].desired - con[psi].state);
				PIFF_wStep(&con[psi],Ts, errorWrap);

				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
				nu->header.stamp = ros::Time::now();
				nu->goal.requester = "fadp_controller";

				Eigen::Vector2f out, in;
				Eigen::Matrix2f R;
				in<<con[x].output,con[y].output;
				double yaw(state.orientation.yaw);
				R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
				out = R.transpose()*in;

				nu->twist.linear.x = out[0];
				nu->twist.linear.y = out[1];
				nu->twist.angular.z = con[psi].output;

				return nu;
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
					PIDBase_init(&con[i]);
					PIFF_tune(&con[i], float(closedLoopFreq(i)));
				}

				ROS_INFO("Dynamic positioning controller initialized.");
			}

		private:
			PIDBase con[3];
			ros::Subscriber headingRef;
			double Ts;
			double uff,vff;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"fadp_control");

	labust::control::HLControl<labust::control::FADPControl,
	labust::control::EnableServicePolicy,
	labust::control::WindupPolicy<auv_msgs::BodyForceReq> > controller;
	ros::spin();

	return 0;
}



