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
#ifndef DPCONTROL_HPP_
#define DPCONTROL_HPP_
#include <labust/control/PIDController.h>
#include <labust/math/NumberManipulation.hpp>
#include <labust_uvapp/EnableControl.h>

#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyForceReq.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the implementation of the station keeping controller for a
		 * non-holonomic surface vehicle.
		 *
		 * \todo Split into policies 2D/3D non-holonomic, 2D/3D holonomic, station keep, tracking, PID, backstepping
		 */
		class DPControl
		{
		public:
			/**
			 * Main constructor
			 */
			DPControl();
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
			void onNewPoint(const geometry_msgs::PointStamped::ConstPtr& point);
			/**
			 * Handle the new tracking point.
			 */
			void onTrackPoint(const auv_msgs::NavSts::ConstPtr& ref);
			/**
			 * Handle incoming estimates message.
			 */
			void onEstimate(const auv_msgs::NavSts::ConstPtr& estimate);
			/**
			 * Handle windup occurence.
			 */
			void onWindup(const auv_msgs::BodyForceReq::ConstPtr& tauAch);
			/**
			 * Handle the enable control request.
			 */
			bool onEnableControl(labust_uvapp::EnableControl::Request& req,
					labust_uvapp::EnableControl::Response& resp);
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
			 * The horizontal distance PD controller. It has a
			 * limit on the P output value.
			 */
			PIDController headingController, distanceController;
			/**
			 * The sampling time.
			 */
			double Ts, safetyRadius;
			/**
			 * Last received vehicle state.
			 */
			auv_msgs::NavSts state, trackPoint;
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
			ros::Subscriber stateHat, refPoint, refTrack, enableFlag, windup;
			/**
			 * The heading unwrapper.
			 */
			labust::math::unwrap yaw_ref;
			/**
			 * High level controller service.
			 */
			ros::ServiceServer enableControl;
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
