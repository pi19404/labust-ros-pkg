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
#ifndef SIMSENSORS_HPP_
#define SIMSENSORS_HPP_
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace labust
{
	namespace simulation
	{
		class RBModel;

		/**
		 * The interface class for the different simulated sensors.
		 */
		class SimSensorInterface
		{
		public:
			struct Hook
			{
				Hook(const RBModel& model,
						tf::TransformBroadcaster& broadcaster,
						tf::TransformListener& listener,
						bool noisy = false):
							model(model),
							broadcaster(broadcaster),
							listener(listener),
							noisy(noisy){};

				const RBModel& model;
				tf::TransformBroadcaster& broadcaster;
				tf::TransformListener& listener;
				bool noisy;
			};

			typedef boost::shared_ptr<SimSensorInterface> Ptr;
			virtual ~SimSensorInterface(){};
			virtual void step(const Hook& data) = 0;
			virtual void configure(ros::NodeHandle& nh, const std::string& topic_name) = 0;
		};

		template <class ROSMsg, class functor>
		class BasicSensor : public SimSensorInterface
		{
		public:
			BasicSensor(){};
			~BasicSensor(){};

			void configure(ros::NodeHandle& nh, const std::string& topic_name)
			{
				pub = nh.advertise<ROSMsg>(topic_name,1);
			};

			void step(const Hook& data)
			{
				typename ROSMsg::Ptr msg(new ROSMsg());
				sensor(msg, data);
				pub.publish(msg);
			}

		private:
				ros::Publisher pub;
				functor sensor;
		};
	}
}

/* SIMSENSORS_HPP_ */
#endif
