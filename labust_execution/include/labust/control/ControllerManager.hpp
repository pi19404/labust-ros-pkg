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
*********************************************************************/
#ifndef CONTROLLERMANAGER_HPP_
#define CONTROLLERMANAGER_HPP_

#include <navcon_msgs/ControllerSelect.h>
#include <navcon_msgs/RegisterController_v2.h>
#include <ros/ros.h>

#include <boost/array.hpp>

#include <string>
#include <map>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the temporary implementation of a controller manager.
		 */
		class ControllerManager
		{
			typedef std::map<std::string,
					navcon_msgs::RegisterController_v2::Request> ControllerMap;

			enum {dofsNum=6};

		public:
			/**
			 * Main constructor
			 */
			ControllerManager();

			/**
			 * The default initialization.
			 */
			void onInit();

		protected:
			/**
			 * Handle the activation request.
			 */
			bool onActivateController(navcon_msgs::ControllerSelect::Request& req,
					navcon_msgs::ControllerSelect::Response& resp);
			/**
			 * Handle the registration request.
			 */
			bool onRegisterController(navcon_msgs::RegisterController_v2::Request& req,
					navcon_msgs::RegisterController_v2::Response& resp);

			/**
			 * Activate the controller.
			 * @param name
			 */
			bool activate(const std::string& name);
			/**
			 * Deactivate the controller.
			 * @param name
			 */
			bool deactivate(const std::string& name);

			/**
			 * Activation service.
			 */
			ros::ServiceServer activateController, registerController;

			/**
			 * The controller state topic publisher.
			 */
			ros::Publisher controllerState;
			/**
			 * The current active tau and nu.
			 */
			boost::array<std::string, 6> actTau, actNu;
			/**
			 * The registered controllers.
			 */
			ControllerMap controllers;
			/**
			 *
			 */
		};
	}
}

/* CONTROLLERMANAGER_HPP_ */
#endif
