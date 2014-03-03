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
#ifndef PRIMITIVEBASE_HPP_
#define PRIMITIVEBASE_HPP_
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <auv_msgs/NavSts.h>
#include <navcon_msgs/ControllerSelect.h>

namespace labust
{
	namespace control
	{
		/**
		 * The base class for course keeping behaviours.
		 */
    template <class ActionType>
		class ExecutorBase
		{
		protected:
    	typedef ActionType Action;
			typedef actionlib::SimpleActionServer<Action> ActionServer;
			typedef boost::shared_ptr<ActionServer> ActionServerPtr;

			/**
			 * Main constructor
			 */
			ExecutorBase(const std::string& name):
				primitiveName(name){};
			/**
			 * The name identifier.
			 */
			std::string primitiveName;
			/**
			 * The publisher of the reference for controllers.
			 */
			ros::Publisher stateRef;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber stateHat;
		  /**
		   * The identification action server.
		   */
		  ActionServerPtr aserver;
		  /**
		   * The service client for controller activation/deactivation
		   */
		  ros::ServiceClient control_manager;
		};

		template <class Executor,
			class OutputType = auv_msgs::NavSts,
			class StateType = auv_msgs::NavSts>
		class PrimitiveBase : public Executor
		{
			typedef PrimitiveBase<Executor,OutputType,StateType> Base;
		public:
			/**
			 * Main constructor
			 */
			PrimitiveBase(){this->onInit();};
			/**
			 * Initialize and setup controller.
			 */
			void onInit()
			{
				ros::NodeHandle nh,ph("~");
				this->aserver.reset(new typename Executor::ActionServer(nh,
						this->primitiveName,	false));

			  this->aserver->registerGoalCallback(
			  		boost::bind(&Base::onGoal, this));
			  this->aserver->registerPreemptCallback(
			  		boost::bind(&Base::onPreempt, this));

				this->stateRef = nh.advertise<OutputType>("out", 1);
				this->stateHat = nh.subscribe<StateType>("state", 1,
						boost::bind(&Base::onStateHat,this,_1));

				this->control_manager =
						nh.serviceClient<navcon_msgs::ControllerSelect>("controller_select", true);

				Executor::init();

				this->aserver->start();
			}
		};
	}
}

/* PRIMITIVEBASE_HPP_ */
#endif
