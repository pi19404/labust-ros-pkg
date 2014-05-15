/*********************************************************************
 * mission_exec.cpp
 *
 *  Created on: Mar 24, 2014
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

#include <labust_mission/labustMission.hpp>
#include <labust_mission/controllerManager.hpp>
#include <labust_mission/missionExecution.hpp>

#include <tinyxml2.h>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;
using namespace tinyxml2;
using namespace labust::mission;

namespace ser = ros::serialization;

/*********************************************************************
*** Global variables
*********************************************************************/

EventQueue* mainEventQueue;
ros::NodeHandle *nh_ptr;
ControllerManager* CM = NULL;
MissionExecution* ME = NULL;


struct MainEventQueue{
MainEventQueue(){ mainEventQueue = new RosEventQueue(); }
~MainEventQueue(){ delete mainEventQueue; }
};

/*********************************************************************
*** Finite State Machine
*********************************************************************/

/* Mission selection  */
FSM(MissionSelect)
{
	FSM_STATES
	{
		Wait_state,
		Dispatcher_state,
		go2point_FA_state,
		go2point_UA_state,
		dynamic_positioning_state,
		course_keeping_FA_state,
		course_keeping_UA_state
	}
	FSM_START(Wait_state);
	FSM_BGN
	{
		FSM_STATE(Wait_state)
		{
			ROS_ERROR("Mission waiting...");

			FSM_ON_STATE_EXIT_BGN{

				ME->oldPosition.north = CM->Xpos;
				ME->oldPosition.east = CM->Ypos;

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/START_DISPATCHER", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(Dispatcher_state)
		{
			ROS_ERROR("Dispatcher active");

			ME->requestPrimitive();

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/GO2POINT_FA", FSM_NEXT(go2point_FA_state));
				FSM_ON_EVENT("/GO2POINT_UA", FSM_NEXT(go2point_UA_state));
				FSM_ON_EVENT("/DYNAMIC_POSITIONING", FSM_NEXT(dynamic_positioning_state));
				FSM_ON_EVENT("/COURSE_KEEPING_FA", FSM_NEXT(go2point_FA_state));
				FSM_ON_EVENT("/COURSE_KEEPING_UA", FSM_NEXT(go2point_FA_state));
				FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
			}
		}
		FSM_STATE(go2point_FA_state)
		{
			ROS_ERROR("go2point_FA primitive active");

			misc_msgs::Go2PointFA data = ME->deserializePrimitive<misc_msgs::Go2PointFA>(ME->receivedPrimitive.primitiveData);
		   	CM->go2point_FA(true,ME->oldPosition.north,ME->oldPosition.east,data.point.north,data.point.east, data.speed, data.heading, data.victoryRadius);

		   	ME->oldPosition = data.point;

			FSM_ON_STATE_EXIT_BGN{

				CM->go2point_FA(false,0,0,0,0,0,0,0);

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(go2point_UA_state)
		{
			ROS_ERROR("go2point_UA primitive active");

			misc_msgs::Go2PointUA data = ME->deserializePrimitive<misc_msgs::Go2PointUA>(ME->receivedPrimitive.primitiveData);
		   	CM->go2point_UA(true,ME->oldPosition.north,ME->oldPosition.east,data.point.north,data.point.east, data.speed, data.victoryRadius);

		   	ME->oldPosition = data.point;

			FSM_ON_STATE_EXIT_BGN{

				CM->go2point_UA(false,0,0,0,0,0,0);

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(dynamic_positioning_state)
		{
			ROS_ERROR("dynamic_positioning primitive active");

			misc_msgs::DynamicPositioning data = ME->deserializePrimitive<misc_msgs::DynamicPositioning>(ME->receivedPrimitive.primitiveData);
		   	CM->dynamic_positioning(true,data.point.north,data.point.east, data.heading);

		   	ME->oldPosition = data.point;

			FSM_ON_STATE_EXIT_BGN{

				CM->dynamic_positioning(false,0,0,0);

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(course_keeping_FA_state)
		{
			ROS_ERROR("course_keeping_FA primitive active");

			misc_msgs::CourseKeepingFA data = ME->deserializePrimitive<misc_msgs::CourseKeepingFA>(ME->receivedPrimitive.primitiveData);
		   	CM->course_keeping_FA(true,data.course, data.speed, data.heading);

			FSM_ON_STATE_EXIT_BGN{

				CM->course_keeping_FA(false,0,0,0);

				ME->oldPosition.north = CM->Xpos;
				ME->oldPosition.east = CM->Ypos;

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(course_keeping_UA_state)
		{
			ROS_ERROR("course_keeping_UA primitive active");

			misc_msgs::CourseKeepingUA data = ME->deserializePrimitive<misc_msgs::CourseKeepingUA>(ME->receivedPrimitive.primitiveData);
		   	CM->course_keeping_UA(true,data.course, data.speed);

			FSM_ON_STATE_EXIT_BGN{

				CM->course_keeping_UA(false,0,0);

				ME->oldPosition.north = CM->Xpos;
				ME->oldPosition.east = CM->Ypos;

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/STOP", FSM_NEXT(Wait_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
	}
	FSM_END
}

/*********************************************************************
 ***  Main function
 ********************************************************************/

int main(int argc, char** argv){

	ros::init(argc, argv, "ControllerFSM");
	ros_decision_making_init(argc, argv);
	ros::NodeHandle nh;
	nh_ptr = &nh;

	/* Start Mission Execution */
	MissionExecution MissExec(nh);
	ME = &MissExec;

	/* Global event queue */
	MainEventQueue meq;

	/* Start Controller Manager */
	ControllerManager ConMan;
	ConMan.start();
	CM = &ConMan;

	/* Start state machine */
	ros::AsyncSpinner spinner(2);
	spinner.start();
	FsmMissionSelect(NULL, mainEventQueue);
	spinner.stop();

	return 0;
}



