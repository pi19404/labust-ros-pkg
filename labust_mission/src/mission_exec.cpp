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


#include <tinyxml2.h>

#include <decision_making/SynchCout.h>
#include <decision_making/BT.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

#include <std_msgs/Bool.h>
#include <auv_msgs/NED.h>

using namespace std;
using namespace decision_making;
using namespace tinyxml2;

/*********************************************************************
*** Global variables
*********************************************************************/

EventQueue* mainEventQueue;
ros::NodeHandle *nh_ptr;
ControllerManager* CM = NULL;

struct MainEventQueue{
MainEventQueue(){ mainEventQueue = new RosEventQueue(); }
~MainEventQueue(){ delete mainEventQueue; }
};

volatile double newXpos, newYpos, newVictoryRadius, newSpeed, newCourse, newHeading;
volatile double oldXpos, oldYpos, oldVictoryRadius, oldSpeed, oldCourse, oldHeading;

volatile int ID=0;


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

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/START_DISPATCHER", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(Dispatcher_state)
		{
			ROS_ERROR("Dispatcher active");

			FSM_CALL_TASK(dispatcherTask)

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/GO2POINT_FA", FSM_NEXT(go2point_FA_state));
				FSM_ON_EVENT("/GO2POINT_UA", FSM_NEXT(go2point_UA_state));
				FSM_ON_EVENT("/DYNAMIC_POSITIONING", FSM_NEXT(dynamic_positioning_state));
				FSM_ON_EVENT("/COURSE_KEEPING_FA", FSM_NEXT(go2point_FA_state));
				FSM_ON_EVENT("/COURSE_KEEPING_UA", FSM_NEXT(go2point_FA_state));
			}
		}
		FSM_STATE(go2point_FA_state)
		{
			ROS_ERROR("go2point_FA primitive active");

		   	CM->go2point_FA(true,CM->Xpos,CM->Ypos,newXpos,newYpos, newSpeed, newHeading, newVictoryRadius);

			FSM_ON_STATE_EXIT_BGN{

				CM->go2point_FA(false,0,0,0,0,0,0,0);

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/RESET", FSM_NEXT(Dispatcher_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(go2point_UA_state)
		{
			ROS_ERROR("go2point_UA primitive active");

			CM->go2point_UA(true,CM->Xpos,CM->Ypos,newXpos,newYpos, newSpeed, newVictoryRadius);

			FSM_ON_STATE_EXIT_BGN{

				CM->go2point_UA(false,0,0,0,0,0,0);

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/RESET", FSM_NEXT(Dispatcher_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(dynamic_positioning_state)
		{
			ROS_ERROR("dynamic_positioning primitive active");

			CM->dynamic_positioning(true, newXpos, newYpos, newHeading);

			FSM_ON_STATE_EXIT_BGN{

				CM->dynamic_positioning(false,0,0,0);

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/RESET", FSM_NEXT(Dispatcher_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(course_keeping_FA_state)
		{
			ROS_ERROR("course_keeping_FA primitive active");

			CM->course_keeping_FA(true, newCourse, newSpeed, newHeading);

			FSM_ON_STATE_EXIT_BGN{

				CM->course_keeping_FA(false,0,0,0);

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/RESET", FSM_NEXT(Dispatcher_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
		FSM_STATE(course_keeping_UA_state)
		{
			ROS_ERROR("course_keeping_UA primitive active");

			CM->course_keeping_UA(true, newCourse, newSpeed);

			FSM_ON_STATE_EXIT_BGN{

				CM->course_keeping_UA(false,0,0);

			}FSM_ON_STATE_EXIT_END

			FSM_TRANSITIONS
			{
				FSM_ON_EVENT("/RESET", FSM_NEXT(Dispatcher_state));
				FSM_ON_EVENT("/PRIMITIVE_FINISHED", FSM_NEXT(Dispatcher_state));
			}
		}
	}
	FSM_END
}

/*********************************************************************
 ***  ROS Subscriptions Callback
 ********************************************************************/

///\todo vidjeti da li prebaciti ovo u controllerManager ili definirati posebnu klasu za prikupljanje mjerenja i generiranje eventa
void onStateHat(const auv_msgs::NavSts::ConstPtr& data){

}

void onEventString(const std_msgs::String::ConstPtr& msg){

	mainEventQueue->riseEvent(msg->data.c_str());
	ROS_INFO("EventString: %s",msg->data.c_str());
}

/*********************************************************************
 *** Helper functions
 ********************************************************************/

int parseDynamic(int id, string xmlFile){

   XMLDocument xmlDoc;

   XMLNode *mission;
   XMLNode *primitive;
   XMLNode *primitiveParam;

   /* Open XML file */
   if(xmlDoc.LoadFile(xmlFile.c_str()) == XML_SUCCESS) {

	   /* Find mission node */
	   mission = xmlDoc.FirstChildElement("mission");
	   if(mission){

		   /* Loop through primitive nodes */
		   primitive = mission->FirstChildElement("primitive");
		   do{

			   XMLElement *elem = primitive->ToElement();
			   string primitiveName = elem->Attribute("name");
			   ROS_INFO("%s", primitiveName.c_str());

			   primitiveParam = primitive->FirstChildElement("id");
			   XMLElement *elemID = primitiveParam->ToElement();

			   /* If ID is correct process primitive data */
			   string id_string = static_cast<ostringstream*>( &(ostringstream() << id) )->str();
			   string tmp = elemID->GetText();

			   if (tmp.compare(id_string) == 0){

				   /* Case: go2point_FA *****************************/
				   if(primitiveName.compare("go2point_FA") == 0){

					   primitiveParam = primitive->FirstChildElement("param");
					   do{

						   XMLElement *elem2 = primitiveParam->ToElement();
						   string primitiveParamName = elem2->Attribute("name");
						   //ROS_ERROR("%s", primitiveParamName.c_str());

						   if(primitiveParamName.compare("north") == 0){

							   newXpos = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("east") == 0){

							   newYpos = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("speed") == 0){

							   newSpeed = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("victory_radius") == 0){

							   newVictoryRadius = atof(elem2->GetText());
						   } else if(primitiveParamName.compare("heading") == 0){

							   newHeading = atof(elem2->GetText());
						   }
					   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

					   return go2point_FA;

				    /* Case: go2point_UA ****************************/
				    } else if (primitiveName.compare("go2point_UA") == 0){

					   primitiveParam = primitive->FirstChildElement("param");
					   do{

						   XMLElement *elem2 = primitiveParam->ToElement();
						   string primitiveParamName = elem2->Attribute("name");

						   if(primitiveParamName.compare("north") == 0){

							   newXpos = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("east") == 0){

							   newYpos = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("speed") == 0){

							   newSpeed = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("victory_radius") == 0){

							   newVictoryRadius = atof(elem2->GetText());
						   }
					   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

					   return go2point_UA;

				    /* Case: dynamic_positioning ********************/
				    } else if (primitiveName.compare("dynamic_positioning") == 0){

					   primitiveParam = primitive->FirstChildElement("param");
					   do{

						   XMLElement *elem2 = primitiveParam->ToElement();
						   string primitiveParamName = elem2->Attribute("name");
						   //ROS_ERROR("%s", primitiveParamName.c_str());

						   if(primitiveParamName.compare("north") == 0){

							   newXpos = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("east") == 0){

							   newYpos = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("heading") == 0){

							   newHeading = atof(elem2->GetText());
						   }
					   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

			   			return dynamic_positioning;

			   	    /* Case: course_keeping_FA **********************/
			   	    }else if (primitiveName.compare("course_keeping_FA") == 0){

			   		   primitiveParam = primitive->FirstChildElement("param");
					   do{

						   XMLElement *elem2 = primitiveParam->ToElement();
						   string primitiveParamName = elem2->Attribute("name");
						   //ROS_ERROR("%s", primitiveParamName.c_str());

						   if(primitiveParamName.compare("course") == 0){

							   newCourse = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("speed") == 0){

							   newSpeed = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("heading") == 0){

							   newHeading = atof(elem2->GetText());
						   }
					   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

			   		   return course_keeping_FA;

			   		/* Case: course_keeping_UA **********************/
		   		    }else if (primitiveName.compare("course_keeping_UA") == 0){

		   			   primitiveParam = primitive->FirstChildElement("param");
					   do{

						   XMLElement *elem2 = primitiveParam->ToElement();
						   string primitiveParamName = elem2->Attribute("name");
						   //ROS_ERROR("%s", primitiveParamName.c_str());

						   if(primitiveParamName.compare("course") == 0){

							   newCourse = atof(elem2->GetText());

						   } else if(primitiveParamName.compare("speed") == 0){

							   newSpeed = atof(elem2->GetText());
						   }
					   } while(primitiveParam = primitiveParam->NextSiblingElement("param"));

			   		   return course_keeping_UA;
		   		  }
			   }
		   } while(primitive = primitive->NextSiblingElement("primitive"));

		   return none;

	   } else {
		   ROS_ERROR("No mission defined");
	   }
   } else {
	   ROS_ERROR("Cannot open XML file!");
   }
}

/*********************************************************************
 ***  Local Tasks
 ********************************************************************/

/* Dispatcher Task */
decision_making::TaskResult dispatcherTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {

	ros::NodeHandle ph("~");
	string xmlFile = "mission.xml";
	ph.param("xml_save_path", xmlFile, xmlFile);

	ROS_ERROR("%s",xmlFile.c_str());

	ID++;
	int status = parseDynamic(ID, xmlFile);

	ROS_ERROR("%s", primitives[status]);

	switch(status){

		case go2point_FA:

			ROS_ERROR("T1 = %f,%f, T2 = %f,%f, Heading = %f, Speed = %f, Victory radius = %f", CM->Xpos, CM->Ypos, newXpos, newYpos, newHeading, newSpeed, newVictoryRadius);
			mainEventQueue->riseEvent("/GO2POINT_FA");
			break;

		case go2point_UA:

			ROS_ERROR("T1 = %f,%f, T2 = %f,%f, Speed = %f, Victory radius = %f", CM->Xpos, CM->Ypos, newXpos, newYpos, newSpeed, newVictoryRadius);
			mainEventQueue->riseEvent("/GO2POINT_UA");
			break;

		case dynamic_positioning:

			ROS_ERROR("T2 = %f,%f, Heading = %f", newXpos, newYpos, newHeading);
			mainEventQueue->riseEvent("/DYNAMIC_POSITIONING");
			break;

		case course_keeping_FA:
			ROS_ERROR("Course = %f, Heading = %f, Speed = %f", newCourse, newHeading, newSpeed);
			mainEventQueue->riseEvent("/COURSE_KEEPING_FA");
			break;

		case course_keeping_UA:

			ROS_ERROR("Course = %f, Speed = %f", newCourse, newSpeed);
			mainEventQueue->riseEvent("/COURSE_KEEPING_UA");
			break;

		case none:

			ROS_ERROR("Mission ended.");
	}

    return TaskResult::SUCCESS();
}


/*********************************************************************
 ***  Main function
 ********************************************************************/

int main(int argc, char** argv){

	ros::init(argc, argv, "ControllerFSM");
	ros_decision_making_init(argc, argv);
	ros::NodeHandle nh;
	nh_ptr = &nh;

	/* Subscribers */
	ros::Subscriber subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1,onStateHat);
	ros::Subscriber subEventString = nh.subscribe<std_msgs::String>("eventString",1,onEventString);

	/* Publishers */

	/* Tasks registration */
	LocalTasks::registrate("dispatcherTask", dispatcherTask);

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



