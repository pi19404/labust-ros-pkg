/*********************************************************************
 * EKF3D.cpp
 *
 *  Created on: 
 *      Author: Dula Nad
 *      
 *  Modified by: Filip Mandic
 *
 ********************************************************************/
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, LABUST, UNIZG-FER
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
#ifndef EKF3D_HPP_
#define EKF3D_HPP_
#include <labust/navigation/KFCore.hpp>
#include <labust/navigation/LDTravModelExtended.hpp>
#include <labust/navigation/SensorHandlers.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <navcon_msgs/ModelParamsUpdate.h>

#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <auv_msgs/BodyForceReq.h>

#include <auv_msgs/NED.h> // Dodano
#include <std_msgs/Bool.h>

namespace labust
{
	namespace navigation
	{
		/**
	 	 * The 3D state estimator for ROVs and AUVs. Extended with altitude and pitch estimates.
	 	 *
	 	 * \todo Extract the lat/lon part into the llnode.
	 	 */
		class Estimator3D
		{
			enum{X=0,Y,Z,K,M,N, DoF};
			typedef labust::navigation::KFCore<labust::navigation::LDTravModel> KFNav;
		public:
			/**
			 * Main constructor.
			 */
			Estimator3D();

			/**
			 * Initialize the estimation filter.
			 */
			void onInit();
			/**
			 * Start the estimation loop.
			 */
			void start();

		private:
			/**
			 * Helper function for navigation configuration.
			 */
			void configureNav(KFNav& nav, ros::NodeHandle& nh);
			/**
			 * On model updates.
			 */
			void onModelUpdate(const navcon_msgs::ModelParamsUpdate::ConstPtr& update);
			/**
			 * Handle input forces and torques.
			 */
			void onTau(const auv_msgs::BodyForceReq::ConstPtr& tau);
			/**
			 * Handle the depth measurement.
			 */
			void onDepth(const std_msgs::Float32::ConstPtr& data);
			/**
			 * Handle the altitude measurement.
			 */
			void onAltitude(const std_msgs::Float32::ConstPtr& data);
			/**
			 * Helper method to process measurements.
			 */
			void processMeasurements();
			/**
			 * Helper method to publish the navigation state.
			 */
			void publishState();
			/**
			 * Handle the state reset.
			 */
			void onReset(const std_msgs::Bool::ConstPtr& reset);
			/**
			 * Handle the gyro/compass switch.
			 */
			void onUseGyro(const std_msgs::Bool::ConstPtr& use_gyro);

			/**
			 * The navigation filter.
			 */
			KFNav nav;
			/**
			 * The input vector.
			 */
			KFNav::vector tauIn;
			/**
			 * The measurements vector and arrived flag vector.
			 */
			KFNav::vector measurements, newMeas;
			/**
			 * Heading unwrapper.
			 */
			labust::math::unwrap unwrap;
			/**
			 * Estimated and measured state publisher.
			 */
			ros::Publisher stateMeas, stateHat, currentsHat, buoyancyHat;
			/**
			 * Sensors and input subscribers.
			 */
			ros::Subscriber tauAch, depth, altitude, modelUpdate, resetTopic, useGyro, sub, subKFmode;
			/**
			 * The GPS handler.
			 */
			GPSHandler gps;
			/**
			 * The Imu handler.
			 */
			ImuHandler imu;
			/**
			 * The DVL handler.
			 */
			DvlHandler dvl;
			/**
			 * The transform broadcaster.
			 */
			tf2_ros::TransformBroadcaster broadcaster;
			/**
			 * Temporary altitude storage.
			 */
			double alt;
			/**
			 * Model parameters
			 */
			KFNav::ModelParams params[DoF];
			/**
			 * The flag to indicate existing yaw-rate measurements.
			 */
			bool useYawRate;
			/**
			 * The DVL model selector.
			 */
			int dvl_model;
			/**
			 * The compass and gyro variance.
			 */
			double compassVariance, gyroVariance;

			/**
			 * Callbacks for relative/absolute mode switching
			 */
			void deltaPosCallback(const auv_msgs::NED::ConstPtr& msg);

			void KFmodeCallback(const std_msgs::Bool::ConstPtr& msg);

			/**
			 * Variables for relative/absolute mode switching
			 */
			bool quadMeasAvailable, KFmode, KFmodePast, absoluteEKF;

			float deltaXpos, deltaYpos;

			KFNav::matrix Pstart, Rstart;

		};
	}
}
/* EKF3D_HPP_ */
#endif
