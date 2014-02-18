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
#ifndef IDENTIFICATIONNODE_HPP_
#define IDENTIFICATIONNODE_HPP_
#include <labust/control/SOIdentification.hpp>
#include <labust/simulation/matrixfwd.hpp>

#include <auv_msgs/NavSts.h>
#include <actionlib/server/simple_action_server.h>
#include <navcon_msgs/DOFIdentificationAction.h>
#include <ros/ros.h>

#include <Eigen/Dense>

namespace labust
{
	namespace control
	{
		/**
		 * The class contains the implementation of the SO identification algorithm in the ROS framework.
		 *
		 * \todo Set feedback value in ident action.
		 * \todo Make SOIdentification more user friendly C++ class.
		 * \todo Make SOIdentification more simple/ C implementable for uC.
		 */
		class IdentificationNode
		{
			enum {x=0,y,z,roll,pitch,yaw,altitude, u, v, w, measNum};
			enum {X=0,Y,Z,K,M,N};
			enum {alpha=0,beta,betaa};

			typedef navcon_msgs::DOFIdentificationAction Action;
			typedef actionlib::SimpleActionServer<Action> ActionServer;
			typedef boost::shared_ptr<ActionServer> ActionServerPtr;
			typedef navcon_msgs::DOFIdentificationGoal Goal;
			typedef navcon_msgs::DOFIdentificationResult Result;

			typedef Eigen::Matrix<double, measNum,1> MeasVec;

		public:
			/**
			 * Main constructor
			 */
			IdentificationNode();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

		private:
			/**
			 * Handle incoming estimates message.
			 */
			void onMeasurement(const auv_msgs::NavSts::ConstPtr& meas);
			/**
			 * Execute the identification loop.
			 */
			void doIdentification(const Goal::ConstPtr& goal);
			/**
			 * Publish the force and torque values.
			 */
			void setTau(int elem, double value);

			/**
			 * The publisher of the TAU message.
			 */
			ros::Publisher tauOut;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber meas;
		  /**
		   * The identification action server.
		   */
		  ActionServerPtr aserver;
		  /**
		   * The identification thread.
		   */
		  boost::thread identrunner;
		  /**
		   * The measurement vector.
		   */
		  MeasVec measurements;
		  /**
		   * Integration based x,y.
		   * The x,y measurements will be calculated by
		   * integration of forward and lateral speed
		   * measurements.
		   */
		  bool integrateUV;
		  /**
		   * Use surge-sway error.
		   * The difference between the desired speed and measured will be
		   * integrated as the error to the identification.
		   */
		  bool useUV;
		  /**
		   * Use heave error.
		   * The difference between the desired heave and measured will be
		   * integrated as the error to the identification.
		   */
		  bool useW;
		  /**
		   * Cumulative error.
		   */
		  double cumulative_error;
		  /**
		   * The measurement mutex.
		   */
		  boost::mutex measmux;
		};
	}
}

/* IDENTIFICATIONNODE_HPP_ */
#endif
