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
#ifndef SIMCORE_HPP_
#define SIMCORE_HPP_
#include <labust/simulation/RBModel.hpp>
#include <labust/ros/SimSensors.hpp>
#include <labust/tools/conversions.hpp>

#include <auv_msgs/BodyForceReq.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <auv_msgs/NavSts.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/thread.hpp>

namespace labust
{
	namespace simulation
	{
		/**
		 * The method populates the rigid body model from the
		 *
		 * \param nh The node handle to the configuration.
		 * \param model The rigid body model to configure.
		 */
		void configureModel(const ros::NodeHandle& nh, RBModel& model);

		/**
		 *  This class implements core functionality of the ROS uvsim node.
		 *
		 *  \todo Split into policy NavSts or Odom
		 *  \todo Push the allocation part into the RBModel class.
		 */
		class SimCore
		{
			///Allocation types
			typedef enum {X=0,diff} alloc_type;
		public:
			/**
			 * The generic constructor.
			 */
			SimCore();

			/**
			 * The method initializes the ROS node and configures the model from the ROS parameters.
			 */
			void onInit();

			/**
			 * Start the execution.
			 */
			void start();

			/**
			 * Add a new sensor to the simulation.
			 */
			inline void addSensor(SimSensorInterface::Ptr sensor)
			{
					boost::mutex::scoped_lock l(sensor_mux);
					sensors.push_back(sensor);
			}

		private:
			/**
			 * Output the model parameters info.
			 */
			void modelReport();
			/**
			 * Make one simulation step.
			 */
			void step();
			/**
			 * Received the desired tau and calculates the achieved tau based on
			 * the allocation result.
			 */
			template <class ROSMsg>
			void onTau(const typename ROSMsg::ConstPtr& msg)
			{
				boost::mutex::scoped_lock l(tau_mux);
				labust::tools::pointToVector(msg->wrench.force, tau);
				labust::tools::pointToVector(msg->wrench.torque, tau, 3);

				//Allocate
				vector tauAch(tau);
				model.allocator.allocate(tau,tauAch);
				tau = tauAch;

				//Publish allocated.
				typename ROSMsg::Ptr ach(new ROSMsg());
				labust::tools::vectorToPoint(tau,ach->wrench.force);
				labust::tools::vectorToPoint(tau,ach->wrench.torque,3);
				publish_dispatch(ach);
				//Publish windup ???
			}

			/**
			 * The helper methods for publish dispatch.
			 */
			inline void publish_dispatch(auv_msgs::BodyForceReq::Ptr& tau)
			{
				this->tauAch.publish(tau);
			}
			/**
			 * The helper methods for publish dispatch.
			 */
			inline void publish_dispatch(geometry_msgs::WrenchStamped::Ptr& tau)
			{
				this->tauAchWrench.publish(tau);
			}

			/**
			 * The helper function for allocation.
			 *
			 * \todo Refactor this function to be platform agnostic.
			 */
			void allocate();

			/**
			 * Received the external currents speed that act on the rigid body.
			 */
			void onCurrents(const geometry_msgs::TwistStamped::ConstPtr& currents);

			/**
			 * The method calculates and publishes the needed NavSts.
			 */
			void publishNavSts();
			/**
			 * The helper method to copy data from vector to NavSts.
			 */
			void etaNuToNavSts(const vector& eta, const vector& nu, auv_msgs::NavSts& state);

			/**
			 * The method calculates and publishes the needed Odometry message.
			 */
			void publishOdom();
			/**
			 * The helper method to copy data from vector to Odometry message.
			 */
			void etaNuToOdom(const vector& eta, const vector& nu, nav_msgs::Odometry& state);

			/**
			 * The method publishes the main world frames.
			 */
			void publishWorld();
			/**
			 * The method publishes the base link simulation frame.
			 */
			void publishSimBaseLink();

			/**
			 * The rigid body model implementation.
			 */
			RBModel model;
			/**
			 * The frame transform broadcaster.
			 */
			tf::TransformBroadcaster broadcast;
			/**
			 * The frame transform listener.
			 */
			tf::TransformListener listener;
			/**
			 * Subscriptions to input virtual forces and currents.
			 */
			ros::Subscriber tauIn, tauInWrench, currentsSub;
			/**
			 * Publishing of achieved forces.
			 */
			ros::Publisher tauAch, tauAchWrench, meas, measn, odom, odomn;
			/**
			 * The achieved tau.
			 */
			vector tau;
			/**
			 * The tau and model mutex.
			 */
			boost::mutex tau_mux, model_mux, sensor_mux;
			/**
			 * The runner thread.
			 */
			boost::thread runner;
			/**
			 * The sensor vector.
			 */
			std::vector<SimSensorInterface::Ptr> sensors;
			/**
			 * The simulation rate.
			 */
			ros::Rate rate;
			/**
			 * The simulation internal wrap.
			 */
			int wrap;
			/**
			 * The flag to enable publishing of world frame data.
			 */
			bool enablePublishWorld;
			/**
			 * The flag to enable publishing of world frame data.
			 */
			bool enablePublishSimBaseLink;
			/**
			 * The origin latitude and longitude position.
			 */
			double originLat, originLon;
		};
	}
}

/* SIMCORE_HPP_ */
#endif
