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
#include <labust/control/IPFFController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <auv_msgs/BodyForceReq.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The fully actuated dynamic positioning controller
		///\todo add tracking support
		struct FADPControl : DisableAxis
		{
			enum {x=0,y};

			FADPControl():Ts(0.1), useIP(false){};

			void init()
			{
				ros::NodeHandle nh;
				initialize_controller();
			}

  		void windup(const auv_msgs::BodyForceReq& tauAch)
			{
				//Copy into controller
				//con[x].windup = tauAch.disable_axis.x;
				//con[y].windup = tauAch.disable_axis.y;
  			con[x].extWindup = tauAch.windup.x;
  			con[y].extWindup = tauAch.windup.y;
			};

  		void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state)
  		{
				Eigen::Vector2f out, in;
				Eigen::Matrix2f R;
//				in<<0.5,0;
//				double yaw(state.orientation.yaw);
//				R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
//				out = R*in;
//  			con[x].internalState = out(0);
//  			con[y].internalState = out(1);
				con[x].internalState = 0;
				con[y].internalState = 0;
  			con[x].lastState = state.position.north;
  			con[y].lastState = state.position.east;
  			con[x].lastRef = ref.position.north;
  			con[y].lastRef = ref.position.east;
  			con[x].lastError = ref.position.north - state.position.north;
  			con[y].lastError = ref.position.east - state.position.east;
  			ROS_INFO("Reset: %f %f %f %f", con[x].internalState, con[y].internalState,
  					state.position.north, state.position.east);
  		};

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state)
			{
				con[x].desired = ref.position.north;
				con[y].desired = ref.position.east;

				ROS_INFO("Position desired: %f %f", ref.position.north, ref.position.east);

				///\todo There are more options for this ?
				Eigen::Vector2f out, in;
				Eigen::Matrix2f R;
				in<<ref.body_velocity.x,ref.body_velocity.y;
				double yaw(ref.orientation.yaw);
				R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
				out = R*in;
				double uff = out(x);
				double vff = out(y);

				con[x].state = state.position.north;
				con[y].state = state.position.east;

				if (useIP)
				{
					IPFF_ffStep(&con[x],Ts, uff);
					IPFF_ffStep(&con[y],Ts, vff);
				}
				else
				{
					PIFF_ffStep(&con[x],Ts, uff);
					PIFF_ffStep(&con[y],Ts, vff);
				}

				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
				nu->header.stamp = ros::Time::now();
				nu->goal.requester = "fadp_controller";
				labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);

				//ROS_ERROR("Output %f %f %f %f",uff,vff,con[x].output, con[y].output);

				in<<con[x].output,con[y].output;
				yaw = state.orientation.yaw;
				R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
				out = R.transpose()*in;

				nu->twist.linear.x = out[0];
				nu->twist.linear.y = out[1];

				return nu;
			}

			void initialize_controller()
			{
				ROS_INFO("Initializing dynamic positioning controller...");

				ros::NodeHandle nh;
				Eigen::Vector3d closedLoopFreq(Eigen::Vector3d::Ones());
				labust::tools::getMatrixParam(nh,"dp_controller/closed_loop_freq", closedLoopFreq);
				nh.param("dp_controller/sampling",Ts,Ts);
				nh.param("dp_controller/use_ip",useIP,useIP);

				disable_axis[x] = 0;
				disable_axis[y] = 0;

				enum {Kp=0, Ki, Kd, Kt};
				for (size_t i=0; i<2;++i)
				{
					PIDBase_init(&con[i]);
					PIFF_tune(&con[i], float(closedLoopFreq(i)));
				}

				ROS_INFO("Dynamic positioning controller initialized.");
			}

		private:
			PIDBase con[2];
			double Ts;
			bool useIP;
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



