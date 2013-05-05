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
*  Created on: 03.05.2013.
*  Author: Dula Nad
*********************************************************************/
#ifndef LFCONTROL_HPP_
#define LFCONTROL_HPP_
#include <labust/control/PIDController.hpp>
#include <labust/navigation/LFModel.hpp>

#include <auv_msgs/NavSts.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the implementation of the velocity line following controller.
		 * \todo Add 3D line following support.
		 * \todo Remove dependencies to LFModel class.
		 * \todo Move onNewPoint to service ?
		 * \todo Move enable to service ?
		 * \todo Add external speed/force specification
		 */
		class LFControl
		{
		public:
			/**
			 * Main constructor
			 */
			LFControl();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();
			/**
			 * Performs one iteration.
			 */
			void step();
			/**
			 * Start the controller loop.
			 */
			void start();

		private:
			/**
			 * Handle the new point.
			 */
			void onNewPoint(const geometry_msgs::PointStamped::ConstPtr& ref);
			/**
			 * Handle incoming estimates message.
			 */
			void onEstimate(const auv_msgs::NavSts::ConstPtr& estimate);
			/**
			 * Handle enable flag message.
			 */
			void onEnable(const std_msgs::Bool::ConstPtr& flag);
			/**
			 * Dynamic reconfigure callback.
			 */
			//void dynrec_cb(labust_uvapp::VelConConfig& config, uint32_t level);
			/**
			 * The safety test.
			 */
			//void safetyTest();
			/**
			 * Update the dynamic reconfiguration settings.
			 */
			//void updateDynRecConfig();
			/**
			 * The ROS node handles.
			 */
			ros::NodeHandle nh,ph;
			/**
			 * Last message times.
			 */
			ros::Time lastEst;
			/**
			 * Timeout
			 */
			double timeout;

			/**
			 * Initialize the controller parameters etc.
			 */
			void initialize_controller();
			/**
			 * Adjust the controller based on surge speed.
			 */
			void adjustDH();

			/**
			 * The horizontal distance PD controller. It has a
			 * limit on the P output value.
			 */
			lfPD dh_controller;
			/**
			 * The sampling time.
			 */
			double Ts, surge, currSurge, wh, currYaw;
			/**
			 * The line description.
			 */
			labust::navigation::LFModel::Line line;
			/**
			 * Last received vehicle state.
			 */
			labust::navigation::LFModel::vector T0;
			/**
			 * Enabled.
			 */
			bool enable;


			/**
			 * The publisher of the TAU message.
			 */
			ros::Publisher nuRef;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber stateHat, refPoint, enableFlag;
			/**
			 * High level controller service.
			 */
			//ros::ServiceServer highLevelSelect;
			/**
			 * The dynamic reconfigure parameters.
			 */
			//labust_uvapp::VelConConfig config;
			/**
			 * The dynamic reconfigure server.
			 */
		  //dynamic_reconfigure::Server<labust_uvapp::VelConConfig> server;
		};
	}
}

/* VELOCITYCONTROL_HPP_ */
#endif
