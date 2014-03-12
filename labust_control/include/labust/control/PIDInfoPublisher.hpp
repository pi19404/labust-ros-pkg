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
#ifndef PIDINFOPUBLISHER_HPP_
#define PIDINFOPUBLISHER_HPP_
#include <labust/control/PIDBase.h>

#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>

#include <string>

namespace labust
{
	namespace control
	{
		/**
		 * The PIDBase info publisher used for debugging purposes.
		 */
		class PIDInfoPublisher
		{
		public:
			PIDInfoPublisher(ros::NodeHandle nh, const std::string& name = "pid_info")
			{
				infoPub = nh.advertise<std_msgs::Float32MultiArray>(name, 1);
			}

			void operator()(PIDBase& con)
			{
					std_msgs::Float32MultiArray::Ptr info(new std_msgs::Float32MultiArray());
					info->data.push_back(con.Kp);
					info->data.push_back(con.Ki);
					info->data.push_back(con.Kd);
					info->data.push_back(con.Tf);
					info->data.push_back(con.Kt);

					info->data.push_back(con.autoWindup);
					info->data.push_back(con.windup);
					info->data.push_back(con.extWindup);
					info->data.push_back(con.extTrack);

					info->data.push_back(con.outputLimit);
					info->data.push_back(con.internalState);
					info->data.push_back(con.lastRef);
					info->data.push_back(con.lastError);
					info->data.push_back(con.lastFF);
					info->data.push_back(con.lastState);
					info->data.push_back(con.llastError);
					info->data.push_back(con.llastState);
					info->data.push_back(con.lastDerivative);
					info->data.push_back(con.lastI);
					info->data.push_back(con.lastP);

					info->data.push_back(con.desired);
					info->data.push_back(con.state);
					info->data.push_back(con.output);
					info->data.push_back(con.track);

					infoPub.publish(info);
			}

		protected:
			ros::Publisher infoPub;
		};
	}
}

/* PIDINFOPUBLISHER_HPP_ */
#endif
