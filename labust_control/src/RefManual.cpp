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
#include <labust/control/EnablePolicy.hpp>
#include <labust/control/ManControl.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/MatrixLoader.hpp>

#include <auv_msgs/NavSts.h>

#include <boost/thread/mutex.hpp>

namespace labust
{
	namespace control{
		///The manual reference controller
		struct RefManual
		{
			enum {u=0,v,w,p,q,r};
			RefManual():
				nu_max(Eigen::Vector6d::Zero()),
				Ts(0.1),
				stateReady(false),
				stateAcquired(false),
				useFF(false),
				enable(false){this->init();};

			void init()
			{
				ros::NodeHandle nh;
				//Initialize publishers
				stateRef = nh.advertise<auv_msgs::NavSts>("stateRef", 1);

				//Initialize subscribers
				stateHat = nh.subscribe<auv_msgs::NavSts>("stateHat", 1,
						&RefManual::onEstimate,this);
				joyIn = nh.subscribe<sensor_msgs::Joy>("joy", 1,
						&RefManual::onJoy,this);
				
				enableControl = nh.advertiseService("Enable",
						&RefManual::onEnableControl, this);

				initialize_manual();
			}
			
			bool onEnableControl(navcon_msgs::EnableControl::Request& req,
					navcon_msgs::EnableControl::Response& resp)
			{
				this->enable = req.enable;
				if (req.enable)
				{
				  if (stateAcquired)
				  {
					baseRef = lastState;
					stateReady = true;
				  }
				  else
				  {
					stateReady = false;
					return false;
				  }
				}
				else
				{
					stateReady = false;
				}
				
				return true;
			}


			void onEstimate(const auv_msgs::NavSts::ConstPtr& state)
			{
				boost::mutex::scoped_lock l(cnt_mux);
				lastState = *state;
				stateAcquired = true;
				if (!stateReady) baseRef = lastState;
				//Switch occured
				//if (Enable::enable && !stateReady)
				//{
				//	this->baseRef.position = state->position;
				//	this->baseRef.orientation = state->orientation;
				//	this->baseRef.position.depth = state->position.depth;
				//	this->baseRef.altitude = state->altitude;
				//}
				//stateReady = Enable::enable;
			}

			void onJoy(const sensor_msgs::Joy::ConstPtr& joy)
			{
				if (!stateReady) return;
				if (!stateAcquired) return;

				Eigen::Vector6d mapped;
				mapper.remap(*joy, mapped);
				mapped = nu_max.cwiseProduct(mapped);

				Eigen::Vector2f out, in;
				Eigen::Matrix2f R;
				in<<mapped[u]*nu_max[u]*Ts,mapped[v]*nu_max[v]*Ts;
				double yaw(baseRef.orientation.yaw);
				R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
				out = R*in;

				baseRef.header.stamp = ros::Time::now();
				baseRef.header.frame_id = "local";

				baseRef.position.north += out(u);
				baseRef.position.east += out(v);
				baseRef.position.depth += mapped[w]*Ts*nu_max(w);
				baseRef.orientation.roll += mapped[p]*Ts*nu_max(p);
				baseRef.orientation.pitch += mapped[q]*Ts*nu_max(q);
				baseRef.orientation.yaw += mapped[r]*Ts*nu_max(r);
				
				baseRef.altitude -= mapped[w]*Ts*nu_max(w);
				
				if (useFF)
				{
				  baseRef.body_velocity.x = mapped[u]*nu_max[u];
				  baseRef.body_velocity.y = mapped[v]*nu_max[v];
				  baseRef.body_velocity.z = mapped[w]*nu_max[w];
				  baseRef.orientation_rate.roll = mapped[p]*nu_max(p);
				  baseRef.orientation_rate.pitch = mapped[q]*nu_max(q);
				  baseRef.orientation_rate.yaw = mapped[r]*nu_max(r);
				}
				
				auv_msgs::NavSts::Ptr refOut(new auv_msgs::NavSts());
				*refOut = baseRef;
				refOut->orientation.roll =  labust::math::wrapRad(refOut->orientation.roll);
				refOut->orientation.pitch =  labust::math::wrapRad(refOut->orientation.pitch);
				refOut->orientation.yaw =  labust::math::wrapRad(refOut->orientation.yaw);

				stateRef.publish(refOut);
			}

			void initialize_manual()
			{
				ROS_INFO("Initializing manual ref controller...");

				ros::NodeHandle nh;
				labust::tools::getMatrixParam(nh,"ref_manual/maximum_speeds", nu_max);
				nh.param("ref_manual/sampling_time",Ts,Ts);
				nh.param("ref_manual/feedforward_speeds",useFF,useFF);

				ROS_INFO("Manual ref controller initialized.");
			}

		private:
			ros::Subscriber stateHat, joyIn;
			ros::Publisher stateRef;
			Eigen::Vector6d nu_max;
			double Ts;
			JoystickMapping mapper;
			auv_msgs::NavSts baseRef;
			boost::mutex cnt_mux;
			auv_msgs::NavSts lastState;
			bool stateReady;
			bool stateAcquired;
			bool useFF;
			bool enable;
			ros::ServiceServer enableControl;
		};
	}}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"ref_manual");

	//labust::control::RefManual<labust::control::EnableServicePolicy> controller;
	labust::control::RefManual controller;
	ros::Duration(10).sleep();
	ros::spin();

	return 0;
}



