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
#ifndef VELOCITYCONTROL_HPP_
#define VELOCITYCONTROL_HPP_
#include <labust/control/PIDController.h>

#include <auv_msgs/NavSts.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/BodyForceReq.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

#include <string>
#include <map>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the implementation of the velocity controller, manual control manager and model identification.
		 * \todo Add separate joystick scaling for all DOFs
		 * \todo Add identification
		 */
		class VelocityControl
		{
			enum {u=0,v,w,p,q,r};
			enum {X=0,Y,Z,K,M,N};
			enum {alpha=0,beta,betaa};
			enum {Kp=0,Ki,Kd,Kt};

		public:
			/**
			 * Main constructor
			 */
			VelocityControl();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

			/**
			 * Start the controller main loop.
			 */
			void start();
			/**
			 * Stop the controller loop execution
			 */
			inline void stop(){this->runFlag = false;};

			/**
			 * Performs one iteration.
			 */
			void step();

		private:
			/**
			 * Handle incoming reference message.
			 */
			void handleReference(const auv_msgs::BodyVelocityReq::ConstPtr& ref);
			/**
			 * Handle incoming measurement message.
			 */
			void handleMeasurements(const auv_msgs::NavSts::ConstPtr& measurement);
			/**
			 * Handle incoming estimates message.
			 */
			void handleEstimates(const auv_msgs::NavSts::ConstPtr& estimate);
			/**
			 * Handle incoming estimates message.
			 */
			void handleWindup(const auv_msgs::BodyForceReq::ConstPtr& tau);
			/**
			 * Handle incoming estimates message.
			 */
			void handleManual(const sensor_msgs::Joy::ConstPtr& joy);
			/**
			 * Handle incoming estimates message.
			 */
			void handleOpMode(const std_msgs::String::ConstPtr& mode);
			/**
			 * Message updates.
			 */
			bool newReference, newEstimate, newMeasurement;

			/**
			 * The VelocityControl mode.
			 */
			void stepVC(auv_msgs::BodyForceReq& tau);
			/**
			 * The manual control mode.
			 */
			void stepManual(auv_msgs::BodyForceReq& tau);

			/**
			 * Initialize the controller parameters etc.
			 */
			void initialize_controller();
			/**
			 * The velocity controllers.
			 */
			PIDController controller[r+1];
			/**
			 * Joystick message.
			 */
			float tauManual[N+1];
			/**
			 * Joystick scaling.
			 */
			double joy_scale;
			/**
			 * Enable/disable controllers, external windup flag.
			 */
			bool disable_axis[r+1];

			/**
			 * The loop control flag.
			 */
			bool runFlag;

			/**
			 * The publisher of the TAU and status command.
			 */
			ros::Publisher tauOut,windup;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber velocityRef, stateHat, stateMeas, manualIn, tauAch, opMode;
			/**
			 * The ROS node handles.
			 */
			ros::NodeHandle nh,ph;
			/**
			 * The last execution time.
			 */
			ros::Time lastTime;
			/**
			 * Use message or time driven operation.
			 */
			bool synced;
			/**
			 * Current mode.
			 */
			std::string mode;
			/**
			 * The mode reaction map.
			 */
			std::map<std::string, boost::function<void(auv_msgs::BodyForceReq&)> > handler;
		};
	}
}

/* VELOCITYCONTROL_HPP_ */
#endif
