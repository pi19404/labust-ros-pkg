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
#include <labust/tools/conversions.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <auv_msgs/BodyVelocityReq.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

namespace labust
{
	namespace control{
		///The simple line following controller
		struct UALFControl : DisableAxis
		{
			UALFControl():
				Ts(0.1),
				aAngle(M_PI/8),
				wh(0.2),
				underactuated(false),
				listener(buffer){};

			void init()
			{
				initialize_controller();
			}

			void reset(const auv_msgs::NavSts& ref, const auv_msgs::NavSts& state){};

			auv_msgs::BodyVelocityReqPtr step(const auv_msgs::NavSts& ref,
					const auv_msgs::NavSts& state)
			{
				auv_msgs::BodyVelocityReqPtr nu(new auv_msgs::BodyVelocityReq());

				if (ref.header.frame_id != "course_frame")
				{
					ROS_WARN("The reference frame id is not 'course_frame'.");
				}
				//Get the position in the line frame
				try
				{
					geometry_msgs::TransformStamped dH;
					dH = buffer.lookupTransform("course_frame", "base_link", ros::Time(0));
					double roll, pitch, gamma;
					labust::tools::eulerZYXFromQuaternion(dH.transform.rotation,
							roll, pitch, gamma);

					//Calculate desired yaw-rate
					if (underactuated)
					{
						PSatD_tune(&con,wh,aAngle,ref.body_velocity.x);
						double dd = -ref.body_velocity.x*sin(gamma);
						con.desired = ref.position.east;
						con.state = dH.transform.translation.y;
						PSatD_dStep(&con,Ts,dd);
						//PSatD_step(&con,Ts);
						ROS_INFO("Limiter: %f", con.outputLimit);
						nu->twist.angular.z = con.output;
						nu->twist.linear.x = ref.body_velocity.x;
					}
					else
					{
						double ul = ref.body_velocity.x;
						double vl = -0.1*dH.transform.translation.y;
						ROS_INFO("Command output: ul=%f, vl=%f", ul, vl);

						Eigen::Vector2f out, in;
						Eigen::Matrix2f R;
						in<<ul,vl;
						R<<cos(gamma),sin(gamma),-sin(gamma),cos(gamma);
						ROS_INFO("Gamma: %f", gamma);
						out = R*in;
						nu->twist.linear.x = out(0);
						nu->twist.linear.y = out(1);
					}

					ROS_INFO("Command output: cmd=%f, dH=%f, ac=%f", con.output, dH.transform.translation.y,
							dH.transform.translation.x);
				}
				catch (tf2::TransformException& ex)
				{
					ROS_WARN("%s",ex.what());
				}

				nu->header.stamp = ros::Time::now();
				nu->goal.requester = "ualf_controller";
				labust::tools::vectorToDisableAxis(disable_axis, nu->disable_axis);

				return nu;
			}

			void initialize_controller()
			{
				ROS_INFO("Initializing LF controller...");
				ros::NodeHandle nh;
				nh.param("ualf_controller/closed_loop_freq",wh,wh);
				double surge(0.1);
				nh.param("ualf_controller/default_surge",surge,surge);
  			nh.param("ualf_controller/approach_angle",aAngle,aAngle);
				nh.param("ualf_controller/sampling",Ts,Ts);
				nh.param("ualf_controller/underactuated",underactuated,underactuated);

				disable_axis[0] = 0;
				if (underactuated)
				{
					disable_axis[5] = 0;
				}
				else
				{
					disable_axis[1] = 0;
				}

				PIDBase_init(&con);
				PSatD_tune(&con,wh,aAngle,surge);

				ROS_INFO("LF controller initialized.");
			}

		private:
			PIDBase con;
			double Ts;
			double aAngle, wh;
			bool underactuated;
			tf2_ros::Buffer buffer;
			tf2_ros::TransformListener listener;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"lf_control");

	labust::control::HLControl<labust::control::UALFControl,
	labust::control::EnableServicePolicy> controller;
	ros::spin();

	return 0;
}



