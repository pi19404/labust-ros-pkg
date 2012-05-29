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
#ifndef VEHICLEAPP_HPP_
#define VEHICLEAPP_HPP_
#include <labust/vehicles/vehiclesfwd.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The general vehicle plugin template.
		 */
		class VehicleApp
		{
			enum {msgBuf = 10};
		public:
			/**
			 * Main constructor.
			 */
			VehicleApp();

		protected:
			/**
			 * The method loads the requested plugin.
			 *
			 * \param pluginName The name of the plugin library.
			 * \param pluginConfig The config file for driver configuration.
			 */
			void loadPlugin(const std::string& pluginName,
					const std::string& pluginConfig,
					const std::string& pluginId);
			/**
			 * The method handles subscriptions initialization.
			 */
			void subscribe(const std::string& tauName, const std::string& cmdName);
			/**
			 * The method handles publisher initialization.
			 */
			void publish(const std::string& stateName, const std::string& dataName);

			/**
			 * The method handles new tau messages.
			 */
			void onTau(const std_msgs::String::ConstPtr& tau);
			/**
			 * The method handles new tau messages.
			 */
			void onCmd(const std_msgs::String::ConstPtr& cmd);

			/**
			 * The ROS node handle and private handle.
			 */
			ros::NodeHandle nhandle, phandle;
			/**
			 * The publishers.
			 */
			ros::Publisher state, data;
			/**
			 * The subscribers.
			 */
			ros::Subscriber tau, cmd;
			/**
			 * Vehicle plugin.
			 */
			labust::vehicles::VehiclePluginPtr plugin;
			/**
			 * Pointer to the vehicle driver.
			 */
			labust::vehicles::DriverPtr uuv;
		};
	}
}


/* VEHICLEAPP_HPP_ */
#endif
