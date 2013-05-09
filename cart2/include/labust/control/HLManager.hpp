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
*  Created on: 06.05.2013.
*  Author: Dula Nad
*********************************************************************/
#ifndef HLMANAGER_HPP_
#define HLMANAGER_HPP_
#include <cart2/SetHLMode.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <auv_msgs/NavSts.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <map>

namespace labust
{
	namespace control
	{
		/**
		 * The class implements the mission manager for the B-Art and C-Art vehicles.
		 * \todo Look into ROS actionlib to replace this in a more generic resuable way ?
		 * \todo Add controller registration and checking that neccessary controllers run.
		 * \todo Consider making everything async ?
		 */
		class HLManager
		{
			enum {bArt=0, cArt};
			enum {stop=0, manual,
				gotoPoint, stationKeeping, circle};

			typedef std::map<std::string,bool> ControllerMap;

		public:
			/**
			 * Main constructor
			 */
			HLManager();
			/**
			 * Initialize and setup the manager.
			 */
			void onInit();
			/**
			 * Start the manager.
			 */
			void start();

		private:
			/**
			 * Handle vehicle position updates.
			 */
			void onVehicleEstimates(const auv_msgs::NavSts::ConstPtr& estimate);
			/**
			 * The C-Art service handler.
			 */
			bool setHLMode(cart2::SetHLMode::Request& req, cart2::SetHLMode::Response& resp);
			/**
			 * Handle launch detection.
			 */
			void onLaunch(const std_msgs::Bool::ConstPtr& isLaunched);
			/**
			 * On GPS data.
			 */
			void onGPSData(const sensor_msgs::NavSatFix::ConstPtr& fix);
			/**
			 * The safety test.
			 */
			void safetyTest();
			/**
			 * Make one action step.
			 */
			void step();

			/**
			 * The full stop mode.
			 */
			bool fullStop();
			/**
			 * Configure controllers.
			 */
			bool configureControllers();

			/**
			 * The ROS node handles.
			 */
			ros::NodeHandle nh,ph;
			/**
			 * Last message times.
			 */
			ros::Time lastEst, launchTime, lastFix;
			/**
			 * Launch detected flag.
			 */
			bool launchDetected;
			/**
			 * Timeout
			 */
			double timeout;
			/**
			 * The operation mode and vehicle type.
			 */
			int32_t mode, type;
			/**
			 * The reference point.
			 */
			geometry_msgs::PointStamped point;
			/**
			 * The last vehicle state and trajectory specs.
			 */
			auv_msgs::NavSts stateHat, trackPoint;
			/**
			 * The last GPS fix.
			 */
			sensor_msgs::NavSatFix fix;
			/**
			 * GPS fix validated.
			 */
			bool fixValidated;

			/**
			 * The safety radius, distance and time for B-Art.
			 */
			double safetyRadius, safetyDistance, safetyTime;
			/**
			 * Circle parameters
			 */
			double circleRadius, turnDir, lookAhead;
			/**
			 * Local origin.
			 */
			tf::TransformBroadcaster worldLatLon;
			/**
			 * Lat-Lon origin position.
			 */
			double originLat, originLon;

			/**
			 * The publisher of the TAU message.
			 */
			ros::Publisher refPoint, refTrack;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber state, launch, gpsData;
			/**
			 * Mode selector service server.
			 */
			ros::ServiceServer modeServer;
			/**
			 * The highlevel controller set.
			 */
			ControllerMap controllers;
		};
	}
}
/* HLMANAGER_HPP_ */
#endif
