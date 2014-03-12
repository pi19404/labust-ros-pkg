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
#include <navcon_msgs/DynamicPositioningAction.h>
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
	 	///\todo Enable high level controllers using controllerManager class
		///\todo DP underactuated
		struct DPprimitive : protected ExecutorBase<navcon_msgs::DynamicPositioningAction>
		{
			typedef navcon_msgs::DynamicPositioningGoal Goal;
			typedef navcon_msgs::DynamicPositioningResult Result;

			enum {ualf=0,falf,heading, numcnt};

			DPprimitive():
				ExecutorBase("DPprimitive"),
				underactuated(true),
				headingEnabled(false),
				processNewGoal(false){};

			void init()
			{
				ros::NodeHandle ph("~");
//				ph.param("underactuated",underactuated,underactuated);
//
//				controllers.name.resize(numcnt);
//				controllers.state.resize(numcnt, false);
//				std::string temp("ualf");
//				ph.param("ualf_name",temp,temp);
//				controllers.name[ualf] = temp;
//				temp = "falf";
//				ph.param("falf_name",temp,temp);
//				controllers.name[falf] = temp;
//				temp = "heading";
//				ph.param("heading_name",temp,temp);
//				controllers.name[heading] = temp;
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
//				if (new_goal->speed == 0)
//				{
//					ROS_WARN("Cannot perform course keeping without forward speed.");
//					aserver->setAborted(Result(), "Forward speed is zero.");
//				}

				//if ((goal == 0) || (new_goal->course != goal->course))
				if ((goal == 0) || (new_goal->T1.point.x != goal->T1.point.x)
								|| (new_goal->T1.point.y != goal->T1.point.y)
								|| (new_goal->yaw != goal->yaw))
				{
					//Save new goal
					goal = new_goal;
					//ROS_DEBUG("Change course: %f", new_goal->course);
					//Calculate new line if target changed
//					Eigen::Vector3d T1,T2;
//					T1<<lastState.position.north,
//							lastState.position.east,
//							0;
//					T2<<lastState.position.north + 10*cos(new_goal->course),
//							lastState.position.east + 10*sin(new_goal->course),
//							0;
//					line.setLine(T1,T2);
//
//					enum{xp=0,yp,zp};
//					geometry_msgs::TransformStamped transform;
//					transform.transform.translation.x = T1(xp);
//					transform.transform.translation.y = T1(yp);
//					transform.transform.translation.z = T1(zp);
//					labust::tools::quaternionFromEulerZYX(0, 0, line.gamma(),
//							transform.transform.rotation);
//					transform.child_frame_id = "course_frame";
//					transform.header.frame_id = "local";
//					transform.header.stamp = ros::Time::now();
//					broadcaster.sendTransform(transform);

					//Update reference
					//The underactuated controller will auto-enable itself
					stateRef.publish(step(lastState));

					//Enable overactuated controllers
//					if (!underactuated)
//					{
//						controllers.state[falf] = true;
//						controllers.state[heading] = true;
//					}
//					else
//					{
//						double delta = labust::math::wrapRad(lastState.orientation.yaw - line.gamma());
//						ROS_DEBUG("Delta: %f",delta);
//						if (fabs(delta) < M_PI_2)
//						{
//							controllers.state[ualf] = true;
//							controllers.state[heading] = false;
//						}
//					}
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

				/* Enable high level controllers */
				ros::NodeHandle nh;
				ros::ServiceClient cl;

				cl = nh.serviceClient<navcon_msgs::EnableControl>("FADP_enable");
				navcon_msgs::EnableControl a;
				a.request.enable = true;
				cl.call(a);

				cl = nh.serviceClient<navcon_msgs::EnableControl>("HDG_enable");
				a.request.enable = true;
				cl.call(a);
			}

			void onStateHat(const auv_msgs::NavSts::ConstPtr& estimate)
			{
				boost::mutex::scoped_lock l(state_mux);
				if (aserver->isActive())
				{
					stateRef.publish(step(*estimate));
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

				ref->position.north = goal->T1.point.x;
				ref->position.east = goal->T1.point.y;
				//ref->body_velocity.x = goal->speed;
				ref->orientation.yaw = goal->yaw;
				//ref->header.frame_id = "course_frame";
				ref->header.frame_id = "local";

				//Check underactuated behaviour
//				if (underactuated)
//				{
//					ref->orientation.yaw = line.gamma();
//					double delta = labust::math::wrapRad(state.orientation.yaw - line.gamma());
//					ROS_DEBUG("Delta, gamma: %f, %f",delta, line.gamma());
//					if (controllers.state[heading] && (fabs(delta) < M_PI/3))
//					{
//							//disable heading and activate ualf
//							controllers.state[heading] = false;
//							controllers.state[ualf] = true;
//							this->updateControllers();
//							ref->header.frame_id = "course_frame";
//					}
//					else if (fabs(delta) >= M_PI/2)
//					{
//							//deactivate ualf and activate heading
//							controllers.state[heading] = true;
//							controllers.state[ualf] = false;
//							this->updateControllers();
//							ref->header.frame_id = "local";
//					}
//				}

				ref->header.stamp = ros::Time::now();
				return ref;
			}

		private:
			geometry_msgs::Point lastPosition;
			labust::math::Line line;
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
	ros::init(argc,argv,"DPprimitive");

	labust::control::PrimitiveBase<labust::control::DPprimitive> primitive;
	ros::spin();

	return 0;
}



