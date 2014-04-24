//\TODO Typedef kao na line 57
//\TODO Sad kada se koristi typedef moguce realizirati funkcije primitiva kao virtualne funkcije?

/*********************************************************************
 * primitveActionClient.hpp
 *
 *  Created on: Mar 6, 2014
 *      Author: Filip Mandic
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

#ifndef PRIMITIVEACTIONCLIENT_HPP_
#define PRIMITIVEACTIONCLIENT_HPP_

#include <string>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <navcon_msgs/CourseKeepingAction.h>
#include <navcon_msgs/DynamicPositioningAction.h>
#include <navcon_msgs/GoToPointAction.h>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

extern decision_making::EventQueue* mainEventQueue;


namespace utils {

	struct CourseKeepingFA_CB {

		typedef navcon_msgs::CourseKeepingGoal Goal;
		typedef navcon_msgs::CourseKeepingResult Result;
		typedef navcon_msgs::CourseKeepingFeedback Feedback;

		// Called once when the goal completes
			void doneCb(const actionlib::SimpleClientGoalState& state,
					   const Result::ConstPtr& result)
		   {
			 ROS_ERROR("CourseKeepingFA - Finished in state [%s]", state.toString().c_str());
			 mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
		   }

		  // Called once when the goal becomes active
		   void activeCb()
		   {
			 ROS_ERROR("Goal just went active FA");
		   }

		   // Called every time feedback is received for the goal
		   void feedbackCb(const Feedback::ConstPtr& feedback)
		   {
			// ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
			   ROS_ERROR("Feedback");
		   }
	};


	struct CourseKeepingUA_CB {

		typedef navcon_msgs::CourseKeepingGoal Goal;
		typedef navcon_msgs::CourseKeepingResult Result;
		typedef navcon_msgs::CourseKeepingFeedback Feedback;

		// Called once when the goal completes
			void doneCb(const actionlib::SimpleClientGoalState& state,
					   const Result::ConstPtr& result)
		   {
			 ROS_ERROR("CourseKeepingUA - Finished in state [%s]", state.toString().c_str());
			 mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
		   }

		  // Called once when the goal becomes active
		   void activeCb()
		   {
			 ROS_ERROR("Goal just went active UA");
		   }

		   // Called every time feedback is received for the goal
		   void feedbackCb(const Feedback::ConstPtr& feedback)
		   {
			// ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
			   ROS_ERROR("Feedback");
		   }
	};


	struct DPprimitive_CB {

			typedef navcon_msgs::DynamicPositioningGoal Goal;
			typedef navcon_msgs::DynamicPositioningResult Result;
			typedef navcon_msgs::DynamicPositioningFeedback Feedback;

			// Called once when the goal completes
			void doneCb(const actionlib::SimpleClientGoalState& state, const Result::ConstPtr& result)
			   {
				 ROS_ERROR("DPprimitive - Finished in state [%s]", state.toString().c_str());
				 mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
			   }

			  // Called once when the goal becomes active
			void activeCb()
			   {
				 ROS_ERROR("Goal just went active DP");
			   }

			   // Called every time feedback is received for the goal
			void feedbackCb(const Feedback::ConstPtr& feedback)
			   {
				// ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
				   ROS_ERROR("Feedback");
			   }
		};



struct Go2PointFA_CB {

			typedef navcon_msgs::GoToPointGoal Goal;
			typedef navcon_msgs::GoToPointResult Result;
			typedef navcon_msgs::GoToPointFeedback Feedback;

			// Called once when the goal completes
			void doneCb(const actionlib::SimpleClientGoalState& state, const Result::ConstPtr& result)
			   {
				 ROS_ERROR("Go2PointFA - Finished in state [%s]", state.toString().c_str());
				 mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");

			   }

			  // Called once when the goal becomes active
			   void activeCb()
			   {
				 ROS_ERROR("Goal just went active go2point_FA");
			   }

			   // Called every time feedback is received for the goal
			   void feedbackCb(const Feedback::ConstPtr& feedback)
			   {
				// ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
				   ROS_ERROR("Feedback - distance: %f", feedback->distance);
			   }
		};

struct Go2PointUA_CB {

			typedef navcon_msgs::GoToPointGoal Goal;
			typedef navcon_msgs::GoToPointResult Result;
			typedef navcon_msgs::GoToPointFeedback Feedback;

			// Called once when the goal completes
			void doneCb(const actionlib::SimpleClientGoalState& state, const Result::ConstPtr& result)
			   {

				 ROS_ERROR("Go2PointUA - Finished in state [%s]", state.toString().c_str());
				 mainEventQueue->riseEvent("/PRIMITIVE_FINISHED");
			   }

			  // Called once when the goal becomes active
			   void activeCb()
			   {
				 ROS_ERROR("Goal just went active go2point_UA");
			   }

			   // Called every time feedback is received for the goal
			   void feedbackCb(const Feedback::ConstPtr& feedback)
			   {
				// ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
				   ROS_ERROR("Feedback - distance: %f", feedback->distance);
			   }
		};

}


#endif /* PRIMITIVEACTIONCLIENT_HPP_ */
