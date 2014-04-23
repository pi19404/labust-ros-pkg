/*********************************************************************
 * missionExecution.hpp
 *
 *  Created on: Apr 22, 2014
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

#ifndef MISSIONEXECUTION_HPP_
#define MISSIONEXECUTION_HPP_


#include <labust_mission/labustMission.hpp>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

extern decision_making::EventQueue* mainEventQueue;

namespace ser = ros::serialization;

/*********************************************************************
 ***  MissionExecution class definition
 ********************************************************************/

namespace labust {
	namespace mission {

		class MissionExecution{

		public:

			/*****************************************************************
			 ***  Class functions
			 ****************************************************************/

			MissionExecution(ros::NodeHandle& nh);

			/*****************************************************************
			 ***  ROS Subscriptions Callback
			 ****************************************************************/

			//\todo vidjeti da li prebaciti ovo u controllerManager ili definirati posebnu klasu za prikupljanje mjerenja i generiranje eventa
			void onStateHat(const auv_msgs::NavSts::ConstPtr& data);

			void onEventString(const std_msgs::String::ConstPtr& msg);

			void onReceivePrimitive(const misc_msgs::SendPrimitive::ConstPtr& data);

			/*********************************************************************
			 *** Helper functions
			 ********************************************************************/

			template <typename primitiveType>
			primitiveType deserializePrimitive(std::vector<uint8_t> primitiveData);

			void requestPrimitive();

			/*********************************************************************
			 ***  Class variables
			 ********************************************************************/

			misc_msgs::SendPrimitive receivedPrimitive;
			ros::Publisher pubRequestPrimitive;

			ros::Subscriber subStateHatAbs;
			ros::Subscriber subEventString;
			ros::Subscriber subReceivePrimitive;

			auv_msgs::NED oldPosition; /* Remember last primitive end point */

		};

		/*****************************************************************
		 ***  Class functions
		 ****************************************************************/

		MissionExecution::MissionExecution(ros::NodeHandle& nh){

			/* Subscribers */
			subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1, &MissionExecution::onStateHat, this);
			subEventString = nh.subscribe<std_msgs::String>("eventString",1, &MissionExecution::onEventString, this);
			subReceivePrimitive = nh.subscribe<misc_msgs::SendPrimitive>("sendPrimitive",1, &MissionExecution::onReceivePrimitive, this);

			/* Publishers */
			pubRequestPrimitive = nh.advertise<std_msgs::Bool>("requestPrimitive",1);
		}

		/*****************************************************************
		 ***  ROS Subscriptions Callback
		 ****************************************************************/

		//\todo vidjeti da li prebaciti ovo u controllerManager ili definirati posebnu klasu za prikupljanje mjerenja i generiranje eventa
		void MissionExecution::onStateHat(const auv_msgs::NavSts::ConstPtr& data){

		}

		void MissionExecution::onEventString(const std_msgs::String::ConstPtr& msg){

			mainEventQueue->riseEvent(msg->data.c_str());
			ROS_INFO("EventString: %s",msg->data.c_str());
		}

		void MissionExecution::onReceivePrimitive(const misc_msgs::SendPrimitive::ConstPtr& data){

			receivedPrimitive = *data;

			switch(data->primitiveID){

				case go2point_FA:

					mainEventQueue->riseEvent("/GO2POINT_FA");
					break;

				case go2point_UA:

					mainEventQueue->riseEvent("/GO2POINT_UA");
					break;

				case dynamic_positioning:

					mainEventQueue->riseEvent("/DYNAMIC_POSITIONING");
					break;

				case course_keeping_FA:

					mainEventQueue->riseEvent("/COURSE_KEEPING_FA");
					break;

				case course_keeping_UA:

					mainEventQueue->riseEvent("/COURSE_KEEPING_UA");
					break;

				case none:

					ROS_ERROR("Mission ended.");
			}
		}

		/*********************************************************************
		 *** Helper functions
		 ********************************************************************/

		template <typename primitiveType>
		primitiveType MissionExecution::deserializePrimitive(std::vector<uint8_t> primitiveData){

			std::vector<uint8_t> my_buffer;
			primitiveType my_primitive;

			my_buffer = primitiveData;

			uint32_t serial_size = ros::serialization::serializationLength(my_primitive);

			uint8_t *iter = &my_buffer.front();

			ser::IStream stream(iter, serial_size);
			ser::deserialize(stream, my_primitive);

			return my_primitive;
		}

		void MissionExecution::requestPrimitive(){
			std_msgs::Bool req;
			req.data = true;
			pubRequestPrimitive.publish(req);
		}
	}
}

#endif /* MISSIONEXECUTION_HPP_ */
