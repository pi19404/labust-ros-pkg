/*
 * missionExecution.hpp
 *
 *  Created on: Apr 22, 2014
 *      Author: filip
 */

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

class MissionExecution{

public:

	MissionExecution(ros::NodeHandle& nh){

		/* Subscribers */
		subStateHatAbs = nh.subscribe<auv_msgs::NavSts>("stateHatAbs",1, &MissionExecution::onStateHat, this);
		subEventString = nh.subscribe<std_msgs::String>("eventString",1, &MissionExecution::onEventString, this);
		subReceivePrimitive = nh.subscribe<misc_msgs::SendPrimitive>("sendPrimitive",1, &MissionExecution::onReceivePrimitive, this);

		/* Publishers */
		pubRequestPrimitive = nh.advertise<std_msgs::Bool>("requestPrimitive",1);
	}

	/*********************************************************************
	 ***  ROS Subscriptions Callback
	 ********************************************************************/

	//\todo vidjeti da li prebaciti ovo u controllerManager ili definirati posebnu klasu za prikupljanje mjerenja i generiranje eventa
	void onStateHat(const auv_msgs::NavSts::ConstPtr& data){

	}

	void onEventString(const std_msgs::String::ConstPtr& msg){

		mainEventQueue->riseEvent(msg->data.c_str());
		ROS_INFO("EventString: %s",msg->data.c_str());
	}

	void onReceivePrimitive(const misc_msgs::SendPrimitive::ConstPtr& data){

		receivedPrimitive = *data;

		switch(data->primitiveID){

			case go2point_FA:

				ROS_ERROR("Primio primitiv");

				//deserializePrimitive<misc_msgs::Go2PointFA>(data->primitiveData);

				//ROS_ERROR("T1 = %f,%f, T2 = %f,%f, Heading = %f, Speed = %f, Victory radius = %f", CM->Xpos, CM->Ypos, newXpos, newYpos, newHeading, newSpeed, newVictoryRadius);
				mainEventQueue->riseEvent("/GO2POINT_FA");
				break;

			case go2point_UA:

				ROS_ERROR("Primio primitiv");

				//ROS_ERROR("T1 = %f,%f, T2 = %f,%f, Speed = %f, Victory radius = %f", CM->Xpos, CM->Ypos, newXpos, newYpos, newSpeed, newVictoryRadius);
				mainEventQueue->riseEvent("/GO2POINT_UA");
				break;

			case dynamic_positioning:

				//ROS_ERROR("T2 = %f,%f, Heading = %f", newXpos, newYpos, newHeading);
				mainEventQueue->riseEvent("/DYNAMIC_POSITIONING");
				break;

			case course_keeping_FA:
				//ROS_ERROR("Course = %f, Heading = %f, Speed = %f", newCourse, newHeading, newSpeed);
				mainEventQueue->riseEvent("/COURSE_KEEPING_FA");
				break;

			case course_keeping_UA:

				//ROS_ERROR("Course = %f, Speed = %f", newCourse, newSpeed);
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
	primitiveType deserializePrimitive(std::vector<uint8_t> primitiveData){

		std::vector<uint8_t> my_buffer;
		primitiveType my_primitive;

		my_buffer = primitiveData;

		uint32_t serial_size = ros::serialization::serializationLength(my_primitive);

		uint8_t *iter = &my_buffer.front();

		ser::IStream stream(iter, serial_size);
		ser::deserialize(stream, my_primitive);

		return my_primitive;
	}

	void requestPrimitive(){
		std_msgs::Bool req;
		req.data = true;
		pubRequestPrimitive.publish(req);
	}

	/*********************************************************************
	 ***  Local Tasks
	 ********************************************************************/

	/* Dispatcher Task */
//	decision_making::TaskResult dispatcherTask(string name, const FSMCallContext& context, EventQueue& eventQueue) {

//		ros::NodeHandle ph("~");
//		string xmlFile = "mission.xml";
//		ph.param("xml_save_path", xmlFile, xmlFile);
//
//		ROS_ERROR("%s",xmlFile.c_str());
//
//		ID++;
//		int status = 0;
//
//		ROS_ERROR("%s", primitives[status]);
//
//		switch(status){
//
//			case go2point_FA:
//
//				ROS_ERROR("T1 = %f,%f, T2 = %f,%f, Heading = %f, Speed = %f, Victory radius = %f", CM->Xpos, CM->Ypos, newXpos, newYpos, newHeading, newSpeed, newVictoryRadius);
//				mainEventQueue->riseEvent("/GO2POINT_FA");
//				break;
//
//			case go2point_UA:
//
//				ROS_ERROR("T1 = %f,%f, T2 = %f,%f, Speed = %f, Victory radius = %f", CM->Xpos, CM->Ypos, newXpos, newYpos, newSpeed, newVictoryRadius);
//				mainEventQueue->riseEvent("/GO2POINT_UA");
//				break;
//
//			case dynamic_positioning:
//
//				ROS_ERROR("T2 = %f,%f, Heading = %f", newXpos, newYpos, newHeading);
//				mainEventQueue->riseEvent("/DYNAMIC_POSITIONING");
//				break;
//
//			case course_keeping_FA:
//				ROS_ERROR("Course = %f, Heading = %f, Speed = %f", newCourse, newHeading, newSpeed);
//				mainEventQueue->riseEvent("/COURSE_KEEPING_FA");
//				break;
//
//			case course_keeping_UA:
//
//				ROS_ERROR("Course = %f, Speed = %f", newCourse, newSpeed);
//				mainEventQueue->riseEvent("/COURSE_KEEPING_UA");
//				break;
//
//			case none:
//
//				ROS_ERROR("Mission ended.");
//		}
//
//	    return TaskResult::SUCCESS();
//	}

	/*********************************************************************
	 ***  Class variables
	 ********************************************************************/

	//double newXpos, newYpos, newVictoryRadius, newSpeed, newCourse, newHeading;
	//double oldXpos, oldYpos, oldVictoryRadius, oldSpeed, oldCourse, oldHeading;

	misc_msgs::SendPrimitive receivedPrimitive;
	ros::Publisher pubRequestPrimitive;

	ros::Subscriber subStateHatAbs;
	ros::Subscriber subEventString;
	ros::Subscriber subReceivePrimitive;


};




#endif /* MISSIONEXECUTION_HPP_ */
