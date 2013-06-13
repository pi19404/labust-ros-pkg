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
*  Created on: 13.06.2013.
*  Author: Dula Nad
*********************************************************************/
#ifndef AUGMENTEDACOUSTICS_HPP_
#define AUGMENTEDACOUSTICS_HPP_
#include <auv_msgs/NavSts.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the implementation of the augmented acoustic reality controller developed by
		 * A. Vasilijevic.
		 */
		class AugmentedAcoustics
		{
		public:
			/**
			 * Main constructor
			 */
			AugmentedAcoustics();
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
			 * Handle the target that is tracked.
			 */
			void handleTarget(const auv_msgs::NavSts::ConstPtr& target);
			/**
			 * Handle incoming estimates.
			 */
			void handleEstimates(const auv_msgs::NavSts::ConstPtr& estimate);
			/**
			 * Handle incoming estimates message.
			 */
			void handleManual(const sensor_msgs::Joy::ConstPtr& joy);

			/**
			 * The publisher of the TAU message.
			 */
			ros::Publisher tauOut;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber targetRef, stateHat, manualIn;
		};
	}
}

/* AUGMENTEDACOUSTICS_HPP_ */
#endif
