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
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The heading controller
		struct HDGControl : DisableAxis
		{
			enum {x=0,y};

			HDGControl():Ts(0.1),useIP(false){};

			void init()
			{
				ros::NodeHandle nh;
				initialize_controller();
			}

  		void windup(const auv_msgs::BodyForceReq& tauAch)
			{
				//Copy into controller
				con.windup = tauAch.disable_axis.yaw;
			};

  		void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state)
  		{
  			con.internalState = 0;
  			con.lastState = useIP?unwrap(state.orientation.yaw):state.orientation.yaw;
  		};

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state)
			{
				con.desired = ref.orientation.yaw;
				con.state = (useIP?unwrap(state.orientation.yaw):state.orientation.yaw);

				float errorWrap = labust::math::wrapRad(
					con.desired - con.state);
				//Zero feed-forward
				if (useIP)
				{
					IPFF_wffStep(&con,Ts, errorWrap, 0);
				}
				else
				{
					PIFF_wffStep(&con,Ts, errorWrap, 0);
				}

				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());
				nu->header.stamp = ros::Time::now();
				nu->goal.requester = "hdg_controller";
				labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);

				nu->twist.angular.z = con.output;

				return nu;
			}

			void initialize_controller()
			{
				ROS_INFO("Initializing heading controller...");

				ros::NodeHandle nh;
				double closedLoopFreq(1);
				nh.param("hdg_controller/closed_loop_freq", closedLoopFreq, closedLoopFreq);
				nh.param("hdg_controller/sampling",Ts,Ts);
				nh.param("hdg_controller/use_ip",useIP,useIP);

				disable_axis[5] = 0;

				PIDBase_init(&con);
				PIFF_tune(&con, float(closedLoopFreq));

				ROS_INFO("Heading controller initialized.");
			}

		private:
			ros::Subscriber alt_sub;
			PIDBase con;
			double Ts;
			bool useIP;
			labust::math::unwrap unwrap;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"hdg_control");

	labust::control::HLControl<labust::control::HDGControl,
	labust::control::EnableServicePolicy,
	labust::control::WindupPolicy<auv_msgs::BodyForceReq> > controller;
	ros::spin();

	return 0;
}



