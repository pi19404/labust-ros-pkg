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
 *  Author: Gyula Nagy
 *  Created: 14.11.2013.
 *********************************************************************/
#ifndef BEAMPUBLISHER_HPP_
#define BEAMPUBLISHER_HPP_
#include <labust/navigation/NavQuestMessages.hpp>
#include <labust/tools/conversions.hpp>

#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>

namespace labust
{
	namespace navigation
	{
		/**
		 * Generic beam publisher for DVL devices.
		 */
		template<std::size_t N>
		struct BeamPublisher
		{
			typedef boost::shared_ptr<BeamPublisher<N> > Ptr;
			BeamPublisher(ros::NodeHandle nh, const std::string& topic)
			{
				pub = nh.advertise<std_msgs::Float32MultiArray>(topic,1);
			}

			void operator()(const float vec[N], int quality=50)
			{
				std_msgs::Float32MultiArrayPtr array(
						new std_msgs::Float32MultiArray());
				//Condition speed mm/s -> m/s
				float cond_vec[N];
				for (int i=0; i<3;++i) cond_vec[i]=vec[i]/1000;
				array->data.assign(&cond_vec[0],&cond_vec[N]);
				//array->data.push_back(quality);
				pub.publish(array);
			}

			ros::Publisher pub;
		};

		/**
		 * Generic twist publisher for DVL devices.
		 */
		struct TwistPublisher
		{
			typedef boost::shared_ptr<TwistPublisher> Ptr;
			TwistPublisher(ros::NodeHandle nh, const std::string& topic,
					const std::string& frame_id):
						frame_id(frame_id)
			{
				pub = nh.advertise<geometry_msgs::TwistStamped>(topic,1);
			}

			void operator()(const float vec[3])
			{
				geometry_msgs::TwistStamped::Ptr twist(
						new geometry_msgs::TwistStamped());
				twist->header.stamp=ros::Time();
				twist->header.frame_id=frame_id;
				//Condition speed mm/s -> m/s
				float cond_vec[3];
				for (int i=0; i<3;++i) cond_vec[i]=vec[i]/1000;
				labust::tools::vectorToPoint(cond_vec,twist->twist.linear);
				pub.publish(twist);
			}

			std::string frame_id;
			ros::Publisher pub;
		};
	}
}

//BEAMPUBLISHER_HPP_
#endif
