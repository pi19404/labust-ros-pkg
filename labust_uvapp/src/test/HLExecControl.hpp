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
 *  Created: 11.07.2013.
 *********************************************************************/
#ifndef HLEXECCONTROL_HPP_
#define HLEXECCONTROL_HPP_
#include <labust_uvapp/RegisterController.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

#include <string>
#include <map>

namespace labust
{
	namespace control
	{
		class HLExecControl
		{
		public:
			HLExecControl();

			void onInit();

		private:
			/**
			 * The controller registration handler.
			 */
			bool onRegReq(labust_uvapp::RegisterController::Request& req,
					labust_uvapp::RegisterController::Response& resp);
			/**
			 * The desired body velocities from the controller.
			 */
			void onNuIn(const auv_msgs::BodyVelocityReq::ConstPtr& nu);
			/**
			 * Controller select method.
			 */
			void onControllerSelect(const std_msgs::String::ConstPtr& name);

			/**
			 * Mode selector service server.
			 */
			ros::ServiceServer registerServer;
			/**
			 * The desired speed subscriber.
			 */
			ros::Subscriber nuIn, cntSel;
			/**
			 * The merged speed reference.
			 */
			ros::Publisher nuRef;
			/**
			 * The registered controller map.
			 */
			std::map<std::string, labust_uvapp::ControllerInfo> controllers;
			/**
			 * The received controller nu.
			 */
			std::map<std::string, auv_msgs::BodyVelocityReq> desired;
			/**
			 * The active controllers on nu.
			 */
			boost::array<std::string,6> active_dofs;
		};
	}
};

/* HLEXECCONTROL_HPP_ */
#endif



