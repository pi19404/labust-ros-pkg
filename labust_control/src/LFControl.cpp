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
#include <labust/control/PSatDController.h>
#include <std_msgs/Float32.h>
#include <labust/math/Line.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyVelocityReq.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The simple line following controller
		struct LFControl
		{
			LFControl():Ts(0.1),surge(0),aAngle(M_PI/8),wh(0.5){};

			void init()
			{
				ros::NodeHandle nh;
				openLoopSurge = nh.subscribe<std_msgs::Float32>("open_loop_surge", 1,
								&LFControl::onOpenLoopSurge,this);

				initialize_controller();
			}

			void onOpenLoopSurge(const std_msgs::Float32::ConstPtr& surge)
			{
				this->surge = surge->data;
			}

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state)
			{
				//Calculate new line if target changed
				Eigen::Vector3d T0;
				T0<<ref.position.north,
						ref.position.east,
						ref.position.depth;

				if (T0 != line.getT2())
				{
					Eigen::Vector3d T1;
					T1<<state.position.north,
							state.position.east,
							state.position.depth;
					line.setLine(T1,T0);
				}

				//Calculate desired yaw-rate
				PSatD_tune(&con,wh,aAngle,surge);
				double dd = -surge*sin(state.orientation.yaw- line.gamma());
				con.desired=0;
				con.state = -line.calculatedH(T0(0),T0(1),T0(2));
				PSatD_dStep(&con,Ts,dd);
				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
				nu->twist.angular.z = con.output;

				return nu;
			}

			void initialize_controller()
			{
				ROS_INFO("Initializing LF controller...");
				ros::NodeHandle nh;
				nh.param("lf_controller/closed_loop_freq",wh,wh);
				nh.param("lf_controller/default_surge",surge,surge);
				nh.param("lf_controller/approach_angle",aAngle,aAngle);
				nh.param("lf_controller/sampling",Ts,Ts);

				PIDBase_init(&con);
				PSatD_tune(&con,wh,aAngle,surge);

				ROS_INFO("LF controller initialized.");
			}

		private:
			PIDBase con;
			ros::Subscriber openLoopSurge;
			double Ts;
			double surge, aAngle, wh;
			labust::math::Line line;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"lf_control");

	labust::control::HLControl<labust::control::LFControl,
	labust::control::EnableServicePolicy> controller;
	ros::spin();

	return 0;
}



