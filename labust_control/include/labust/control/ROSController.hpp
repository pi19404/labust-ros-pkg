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
 *  Created on: 26.06.2013.
 *  Author: Dula Nad
 *********************************************************************/
#ifndef ROSCONTROLLER_HPP_
#define ROSCONTROLLER_HPP_
#include <navcon_msgs/ControllerState.h>
#include <navcon_msgs/ControllerInfo.h>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace labust
{
	namespace control
	{
		///\todo Remove checking if anything has changed, it the controller is in the list something has changed
		class ControllerStateTracker
		{
			typedef navcon_msgs::ControllerState CStateT;
			typedef navcon_msgs::ControllerInfo InfoT;
			typedef boost::function< void(InfoT&) > CallbackType;
		public:
			typedef boost::shared_ptr<ControllerStateTracker> Ptr;

			ControllerStateTracker(ros::NodeHandle& nh, ros::NodeHandle& ph)
			{
				enableSub = nh.subscribe<navcon_msgs::ControllerState>("controller_state",1,
						&ControllerStateTracker::onControllerState, this);

				name = ros::this_node::getName();
			}

			void registerCallback(const CallbackType& callback){this->callback = callback;};

			void onControllerState(const navcon_msgs::ControllerState::ConstPtr& cstate)
			{
				//Get index of controller
				ROS_INFO("New controller state. Name: %s",name.c_str());
				CStateT::_name_type::const_iterator it = std::find(cstate->name.begin(), cstate->name.end(), name);
				if (it != cstate->name.end())
				{
					size_t idx = it - cstate->name.begin();
					if (changed(cstate->info[idx], info))
					{
						ROS_INFO("New change.");
						info = cstate->info[idx];
						if (callback != 0)
						{
							callback(info);
						}
						else
						{
							ROS_WARN("Empty callback for controller state tracker.");
						}
					}
				}
			}

		protected:
			/**
			 * Comparison of two info object.
			 */
			bool changed(const InfoT& left, const InfoT& right)
			{
				return (left.reference_topic != right.reference_topic) ||
						(left.state_topic != right.state_topic) ||
						(left.tracking_topic != right.tracking_topic) ||
						(left.state != right.state);
			}

			/**
			 * The topic subscriber.
			 */
			ros::Subscriber enableSub;
			/**
			 * Controller name.
			 */
			std::string name;
			/**
			 * The controller info.
			 */
			InfoT info;
			/**
			 * The on change event callback.
			 */
			CallbackType callback;
		};

		template <
			class TypeInterface,
			class MsgHandler
		>
		class ROSControllerBase : protected MsgHandler
		{
		protected:
			/**
			 * Main constructor.
			 * Controller could be inherited instead of composited.
			 */
			ROSControllerBase(){};

			/**
			 * The initialization and subscriptions.
			 */
			void initBase(ros::NodeHandle nh, ros::NodeHandle ph)
			{
				//Init handlers
				this->nh = nh;
				this->ph = ph;
				//Initialize subscribers
				state = nh.subscribe<typename TypeInterface::StateType>("state", 1,
						&MsgHandler::onState,this);
				reference = nh.subscribe<typename TypeInterface::ReferenceType>("reference", 1,
						&MsgHandler::onReference,this);
				tracking = nh.subscribe<typename TypeInterface::OutputType>("tracking", 1,
						&MsgHandler::onTracking,this);

				//Initialize publishers
				output = nh.advertise<typename TypeInterface::OutputType>("out", 1);
			}

			/**
			 * The node handles.
			 */
			ros::NodeHandle nh, ph;

			/**
			 * The state topic subscriber.
			 */
			ros::Subscriber state;
			/**
			 * The reference topic subscriber.
			 */
			ros::Subscriber reference;
			/**
			 * The tracking topic subscriber.
			 */
			ros::Subscriber tracking;
			/**
			 * The output topic publisher.
			 */
			ros::Publisher output;
		};

		/**
		 * The ticker base.
		 */
		class Ticker
		{
			typedef boost::function<void(void)> TickCallback;

			Ticker(TickCallback& callback):onTick(callback){};

		protected:
			/**
			 * The tick callback.
			 */
			TickCallback onTick;
		};

		template <
			class TypeInterface,
			class Windup>
		class SimpleStateTicker : Ticker, public Windup
		{
		protected:
			/**
			 * The tick callback definition.
			 */
			typedef boost::function<void(void)> TickCallback;

			/**
			 * Main constructor.
			 * Controller could be inherited instead of composited.
			 */
			SimpleStateTicker(TickCallback& callback):
				Ticker(callback),
				cState(new typename TypeInterface::StateType()),
				cReference(new typename TypeInterface::ReferenceType()),
				cTracking(new typename TypeInterface::OutputType()){};

			/**
			 * Handle state messages.
			 */
			void onState(const typename TypeInterface::StateType::ConstPtr& state)
			{
				ROS_INFO("Handle state.");
				cState = state;

				//On state driven control
				if ((cState != 0) && (cReference != 0)) this->onTick();
			}
			/**
			 * Handle reference messages.
			 */
			void onReference(const typename TypeInterface::ReferenceType::ConstPtr& ref)
			{
				ROS_INFO("Handle reference.");
				cReference = ref;
				ROS_WARN("Reference handling not implemented.");
			}
			/**
			 * Handle tracking messages.
			 */
			void onTracking(const typename TypeInterface::OutputType::ConstPtr& track)
			{
				ROS_INFO("Handle tracking.");
				cTracking = track;
				ROS_WARN("Tracking handling not implemented.");
			}

			/**
			 * Current state.
			 */
			typename TypeInterface::StateType::ConstPtr cState;
			/**
			 * Current reference.
			 */
			typename TypeInterface::ReferenceType::ConstPtr cReference;
			/**
			 * Current tracking.
			 */
			typename TypeInterface::OutputType::ConstPtr cTracking;
		};

//		template <class MsgHandler>
//		class RefStateTicker : public MsgHandler<TypeInterface>
//		{
//			typedef typename MsgHandler<typename TypeInterface> HandlerBase;
//		protected:
//			/**
//			 * Main constructor.
//			 */
//			RefStateTicker(typename Controller::Ptr controller):
//
//				controller(controller)
//		  {
//				this->onTick = boost::bind(&RefStateTicker::tick, this);
//		  };
//			/**
//			 * The tick message.
//			 */
//			void tick()
//			{
//				double dTref = (this->cState->header.stamp - this->cReference->header.stamp).toSec();
//				double dTtracking = (this->cState->header.stamp - this->cTracking->header.stamp).toSec();
//				ROS_INFO("Current time difference to state: ref=%f, tracking=%f", dTref, dTtracking);
//				//No tracking
//				typename TypeInterface::OutputType::Ptr out(controller->step(*cReference, *cState));
//			}
//
//			/**
//			 * The underlying controller.
//			 */
//			typename Controller::Ptr controller;
//	  }
//
//
//			/**
//			 * Handle the controller state change.
//			 */
//			void onControllerState(const navcon_msgs::ControllerInfo& info)
//			{
//				//Update topic subscriptions
//				updateTopics(info);
//				//Update the state
//				controller->setState(info.state);
//			}
//
//			void updateTopics(const navcon_msgs::ControllerInfo& info)
//			{
//				if (!info.state_topic.empty() && (state.getTopic() != info.state_topic))
//				{
//					state = nh.subscribe<typename Controller::StateType>(info.state_topic, 1,
//							&ROSController::onState,this);
//				}
//
//				if (!info.reference_topic.empty() && (reference.getTopic() != info.reference_topic))
//				{
//					reference = nh.subscribe<typename Controller::ReferenceType>(info.reference_topic, 1,
//							&ROSController::onReference,this);
//				}
//
//				if (!info.tracking_topic.empty() && (tracking.getTopic() != info.tracking_topic))
//				{
//					tracking = nh.subscribe<typename Controller::OutputType>(info.tracking_topic, 1,
//							&ROSController::onTracking,this);
//				}
//			}
//
//
//
//
//			/**
//			 * The controller state tracker.
//			 */
//			ControllerStateTracker::Ptr cstateTracker;
//		};

		/**
		 * The base class for a ROS enabled controllers.
		 * \todo Consider having a vector for subscriber to compact the representation.
		 * \todo Policy MultiControllerStateTracker or SingleControllerStateTracker no state tracker
		 * \todo Simple pass state value to controller policy, or integrated policy handling
		 * \todo Policy with tracking or without tracking
		 * \todo Policy for windup
		 * \todo Policy for controller tick sync on stateHat, sync on all, etc.
		 * \todo Policy dynamic reconfigure or not
		 * \todo Policy single threaded/multi-threaded
		 * \todo Track sampling behaviour
		 */
		template <class Controller>
		class ROSController
		{
		public:
			/**
			 * Main constructor.
			 * Controller could be inherited instead of composited.
			 */
			ROSController(	typename Controller::Ptr controller):
				controller(controller),
				cState(new typename Controller::StateType()),
				cReference(new typename Controller::ReferenceType()),
				cTracking(new typename Controller::OutputType()),
				currentState(0),
				refSync(false),
				stateSync(false),
				trackSync(false){};

			/**
			 * The controller initialization and subscriptions.
			 */
			void onInit(ros::NodeHandle nh, ros::NodeHandle ph)
			{
				//Init handlers
				this->nh = nh;
				this->ph = ph;
				//Initialize subscribers
				state = nh.subscribe<typename Controller::StateType>("state", 1,
						&ROSController::onState,this);
				reference = nh.subscribe<typename Controller::ReferenceType>("reference", 1,
						&ROSController::onReference,this);
				tracking = nh.subscribe<typename Controller::OutputType>("tracking", 1,
						&ROSController::onTracking,this);

				//Initialize publishers
				output = nh.advertise<typename Controller::OutputType>("out", 1);

				//Initialize the controller state tracker.
				cstateTracker.reset(new ControllerStateTracker(nh,ph));
				cstateTracker->registerCallback(boost::bind(&ROSController::onControllerState, this,_1));

				//Initialize the controller
				controller->init(nh, ph);
			}

		protected:
			/**
			 * The tick message.
			 */
			void onTick()
			{
				if (stateSync && refSync)
				{
					double dTref = (cState->header.stamp - cReference->header.stamp).toSec();
					double dTtracking = (cState->header.stamp - cTracking->header.stamp).toSec();
					ROS_INFO("Current time difference to state: ref=%f, tracking=%f", dTref, dTtracking);

					ROS_INFO("Current state: %d.",currentState);

					if (currentState)
					{
						output.publish(controller->step(*cReference, *cState, *cTracking));
					}
					else
					{
						controller->idle(*cReference, *cState, *cTracking);
					}

					//VERBOSE
					controller->info();

					trackSync = stateSync = refSync = false;
				}
			}
			/**
			 * Handle state messages.
			 */
			void onState(const typename Controller::StateType::ConstPtr& state)
			{
				ROS_INFO("Handle state.");
				cState = state;
				stateSync = true;

				//On state driven control
				this->onTick();
			}
			/**
			 * Handle reference messages.
			 */
			void onReference(const typename Controller::ReferenceType::ConstPtr& ref)
			{
				ROS_INFO("Handle reference.");
				cReference = ref;
				refSync = true;

				this->onTick();
				ROS_WARN("Reference handling not implemented.");
			}
			/**
			 * Handle tracking messages.
			 */
			void onTracking(const typename Controller::OutputType::ConstPtr& track)
			{
				ROS_INFO("Handle tracking.");
				cTracking = track;
				trackSync = true;

				this->onTick();

				ROS_WARN("Tracking handling not implemented.");
			}

			/**
			 * Handle the controller state change.
			 */
			void onControllerState(const navcon_msgs::ControllerInfo& info)
			{
				//Update topic subscriptions
				updateTopics(info);
				//Update the state
				currentState = info.state;
				//controller->setState(info.state);
			}

			void updateTopics(const navcon_msgs::ControllerInfo& info)
			{
				if (!info.state_topic.empty() && (state.getTopic() != info.state_topic))
				{
					state = nh.subscribe<typename Controller::StateType>(info.state_topic, 1,
							&ROSController::onState,this);
				}

				if (!info.reference_topic.empty() && (reference.getTopic() != info.reference_topic))
				{
					reference = nh.subscribe<typename Controller::ReferenceType>(info.reference_topic, 1,
							&ROSController::onReference,this);
				}

				if (!info.tracking_topic.empty() && (tracking.getTopic() != info.tracking_topic))
				{
					tracking = nh.subscribe<typename Controller::OutputType>(info.tracking_topic, 1,
							&ROSController::onTracking,this);
				}
			}

			/**
			 * The node handles.
			 */
			ros::NodeHandle nh, ph;

			/**
			 * The state topic subscriber.
			 */
			ros::Subscriber state;
			/**
			 * The reference topic subscriber.
			 */
			ros::Subscriber reference;
			/**
			 * The tracking topic subscriber.
			 */
			ros::Subscriber tracking;
			/**
			 * The output topic publisher.
			 */
			ros::Publisher output;
			/**
			 * Track variables for sync.
			 */
			bool refSync, stateSync, trackSync;

			/**
			 * Current state.
			 */
			typename Controller::StateType::ConstPtr cState;
			/**
			 * Current reference.
			 */
			typename Controller::ReferenceType::ConstPtr cReference;
			/**
			 * Current tracking.
			 */
			typename Controller::OutputType::ConstPtr cTracking;
			/**
			 * The underlying controller.
			 */
			typename Controller::Ptr controller;
			/**
			 * The controller state tracker.
			 */
			ControllerStateTracker::Ptr cstateTracker;
			/**
			 * The state.
			 */
			int currentState;
		};
	}
}

/* HLCONTROL_HPP_ */
#endif
