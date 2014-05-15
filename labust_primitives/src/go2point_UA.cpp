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
#include <labust/control/PrimitiveBase.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/math/Line.hpp>
#include <labust/tools/conversions.hpp>

#include <Eigen/Dense>
#include <navcon_msgs/GoToPointAction.h>
#include <navcon_msgs/EnableControl.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include <boost/array.hpp>

namespace labust
{
	namespace control{
		///The course keeping action
		///\todo Check what happens during the switch
		///\todo Name remapping of controllers should be implemented similar to ROS remapping ?
		///\todo Add the ability to update heading in fully actuated without full recalculation
		struct GoToPoint : protected ExecutorBase<navcon_msgs::GoToPointAction>
		{
			typedef navcon_msgs::GoToPointGoal Goal;
			typedef navcon_msgs::GoToPointResult Result;
			typedef navcon_msgs::GoToPointFeedback Feedback;

			enum {ualf=0,falf,heading, numcnt};

			GoToPoint():
				ExecutorBase("go2point_UA"),
				underactuated(true),
				headingEnabled(false),
				processNewGoal(false),
				lastDistance(0.0){};

			void init()
			{
				ros::NodeHandle ph("~");
				ph.param("underactuated",underactuated,underactuated);

				controllers.name.resize(numcnt);
				controllers.state.resize(numcnt, false);
				std::string temp("ualf");
				ph.param("ualf_name",temp,temp);
				controllers.name[ualf] = temp;
				temp = "falf";
				ph.param("falf_name",temp,temp);
				controllers.name[falf] = temp;
				temp = "heading";
				ph.param("heading_name",temp,temp);
				controllers.name[heading] = temp;
			}

			void onGoal()
			{
				boost::mutex::scoped_lock l(state_mux);
				ROS_DEBUG("On goal.");
				//Set the flag to avoid disabling controllers on preemption
				processNewGoal = true;
				Goal::ConstPtr new_goal = aserver->acceptNewGoal();
				processNewGoal = false;
				//Check if course keeping is possible.
				if (new_goal->speed == 0)
				{
					ROS_WARN("Cannot perform course keeping without forward speed.");
					aserver->setAborted(Result(), "Forward speed is zero.");
				}

				if ((goal == 0) || (new_goal->T1.point.x != goal->T1.point.x)
								|| (new_goal->T1.point.y != goal->T1.point.y)
								|| (new_goal->T2.point.x != goal->T2.point.x)
								|| (new_goal->T2.point.y != goal->T2.point.y))
				{
					//Save new goal
					goal = new_goal;
					//ROS_DEBUG("Change course: %f", new_goal->course);

					//Calculate new line if target changed
					goalPosition.x = new_goal->T2.point.x;
					goalPosition.y = new_goal->T2.point.y;
					goalRadius = new_goal->radius;

					Eigen::Vector3d T1,T2;
					T1<<new_goal->T1.point.x,
							new_goal->T1.point.y,
							0;
					T2<<new_goal->T2.point.x,
							new_goal->T2.point.y,
							0;
					line.setLine(T1,T2);

					enum{xp=0,yp,zp};
					geometry_msgs::TransformStamped transform;
					transform.transform.translation.x = T1(xp);
					transform.transform.translation.y = T1(yp);
					transform.transform.translation.z = T1(zp);
					labust::tools::quaternionFromEulerZYX(0, 0, line.gamma(),
							transform.transform.rotation);
					transform.child_frame_id = "course_frame";
					transform.header.frame_id = "local";
					transform.header.stamp = ros::Time::now();
					broadcaster.sendTransform(transform);

					//Update reference
					//The underactuated controller will auto-enable itself
					stateRef.publish(step(lastState));

					//Enable overactuated controllers
					if (!underactuated)
					{
						controllers.state[falf] = true;
						controllers.state[heading] = true;
					}
					else
					{
						double delta = labust::math::wrapRad(lastState.orientation.yaw - line.gamma());
						ROS_DEBUG("Delta: %f",delta);
						if (fabs(delta) < M_PI_2)
						{
							controllers.state[ualf] = true;
							controllers.state[heading] = false;
						}
					}
					this->updateControllers();
				}

				//Save new goal
				goal = new_goal;
			}

			void onPreempt()
			{
				ROS_ERROR("Preempted.");
				if (!processNewGoal)
				{
					//ROS_ERROR("Stopping controllers.");
					//controllers.state.assign(numcnt, false);
					//this->updateControllers();
				}
				else
				{
					//ROS_ERROR("New goal processing.");
				}
				aserver->setPreempted();
			};

			void updateControllers()
			{
//				navcon_msgs::ControllerSelectResponse resp;
//				bool isOk = control_manager.call(controllers, resp);
//				isOk = isOk && (controllers.name == resp.name);
//				isOk = isOk && (controllers.state == resp.state);
//				if (!isOk)
//				{	//ConMan.DPcontrol();
//					ROS_WARN("Service call failed.");
//					aserver->setAborted(Result(), "Cannot update controllers state.");
//				}

				ros::NodeHandle nh;

				ros::ServiceClient cl;

				if(underactuated){				
					cl = nh.serviceClient<navcon_msgs::EnableControl>("UALF_enable_1");
				} else {
					cl = nh.serviceClient<navcon_msgs::EnableControl>("UALF_enable");
				}
				navcon_msgs::EnableControl a;
				a.request.enable = controllers.state[ualf] || controllers.state[falf];
				cl.call(a);

				cl = nh.serviceClient<navcon_msgs::EnableControl>("HDG_enable");
				a.request.enable = controllers.state[heading];
				cl.call(a);
			}

			void onStateHat(const auv_msgs::NavSts::ConstPtr& estimate)
			{
				boost::mutex::scoped_lock l(state_mux);
				if (aserver->isActive())
				{
					stateRef.publish(step(*estimate));

					 // Check if goal is completed
					Eigen::Vector3d deltaVictory;
//					deltaVictory<<goalPosition.x-estimate->position.north,
//							goalPosition.y-estimate->position.east,
//							0;
					deltaVictory<<goal->T2.point.x-estimate->position.north,
							goal->T2.point.y-estimate->position.east,
							0;

					distVictory = deltaVictory.norm();
					Ddistance = distVictory - lastDistance;
					lastDistance = distVictory;

					if(distVictory < goalRadius){
						result.position.point.x = estimate->position.north;
						result.position.point.y = estimate->position.east;
						result.distance = distVictory;
						aserver->setSucceeded(result);
					}

					Feedback feedback;
					feedback.distance = deltaVictory.norm();
					aserver->publishFeedback(feedback);
				}
				else if (goal != 0)
				{
						goal.reset();
						ROS_INFO("Stopping controllers.");
						controllers.state.assign(numcnt, false);
						this->updateControllers();
				}

				lastState = *estimate;
			}

			auv_msgs::NavStsPtr step(const auv_msgs::NavSts& state)
			{
				auv_msgs::NavStsPtr ref(new auv_msgs::NavSts());

				ref->position.east = 0;
				ref->body_velocity.x = goal->speed;
				ref->orientation.yaw = goal->yaw;
				ref->header.frame_id = "course_frame";

				Eigen::Vector3d T1,T2;
				T1<<state.position.north,
						state.position.east,
						0;
				T2<<goalPosition.x,
						goalPosition.y,
						0;
				line2.setLine(T1,T2);

				//line3 = line;

				if(fabs(labust::math::wrapRad(line2.gamma() - line.gamma())) > 60*M_PI/180 && Ddistance > 0){

					//ref->orientation.yaw = line2.gamma();
					line = line2;
					ROS_ERROR("Changing course");

				}



				//Check underactuated behaviour
				if (underactuated)
				{
					ref->orientation.yaw = line.gamma();
					double delta = labust::math::wrapRad(state.orientation.yaw - line.gamma());
					ROS_DEBUG("Delta, gamma: %f, %f",delta, line.gamma());
					if (controllers.state[heading] && (fabs(delta) < M_PI/3))
					{
							//disable heading and activate ualf
							controllers.state[heading] = false;
							controllers.state[ualf] = true;
							this->updateControllers();
							ref->header.frame_id = "course_frame";
					}
					else if (fabs(delta) >= M_PI/2)
					{
							//deactivate ualf and activate heading
							controllers.state[heading] = true;
							controllers.state[ualf] = false;
							this->updateControllers();
							ref->header.frame_id = "local";
					}
				}

				ref->header.stamp = ros::Time::now();
				return ref;
			}

			geometry_msgs::Point goalPosition;
			double goalRadius, distVictory, lastDistance, Ddistance;
			Result result;
			//auv_msgs::NavSts lastState;
			//Goal::ConstPtr goal;
		private:

			geometry_msgs::Point lastPosition;
			//geometry_msgs::Point goalPosition;
			labust::math::Line line, line2, line3;
			tf2_ros::StaticTransformBroadcaster broadcaster;
			bool underactuated;
			bool headingEnabled;
			bool processNewGoal;
			Goal::ConstPtr goal;
			auv_msgs::NavSts lastState;
			boost::mutex state_mux;
			navcon_msgs::ControllerSelectRequest controllers;
		};
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"go2point_UA");

	labust::control::PrimitiveBase<labust::control::GoToPoint> primitive;
	ros::spin();

	return 0;
}



