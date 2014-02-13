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
 *********************************************************************/
#ifndef SENSORHANDLERS_HPP_
#define SENSORHANDLERS_HPP_
#include <labust/math/NumberManipulation.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ros/ros.h>

namespace labust
{
	namespace navigation
	{

		struct NewArrived
		{
			NewArrived():isNew(false){};

			inline bool newArrived()
			{
				if (isNew)
				{
					isNew=false;
					return true;
				}
				return false;
			}

		protected:
			bool isNew;
		};
		/**
		 * The GPS handler.
		 */
		class GPSHandler : public NewArrived
		{
		public:
			GPSHandler():listener(buffer){};
			void configure(ros::NodeHandle& nh);

			inline const std::pair<double, double>&
			position() const {return posxy;}

			inline const std::pair<double, double>&
			origin() const {return originLL;}

			inline const std::pair<double, double>&
			latlon() const	{return posLL;}

		private:
			void onGps(const sensor_msgs::NavSatFix::ConstPtr& data);
			std::pair<double, double> posxy, originLL, posLL;
			tf2_ros::Buffer buffer;
			tf2_ros::TransformListener listener;
			ros::Subscriber gps;
		};

		/**
		 * The imu handler.
		 */
		class ImuHandler : public NewArrived
		{
		public:
			enum {roll=0, pitch, yaw};
			enum {p=0,q,r};
			enum {ax,ay,az};

			ImuHandler():listener(buffer){};

			void configure(ros::NodeHandle& nh);

			inline const double* orientation() const{return rpy;}

			inline const double* rate() const{return pqr;}

			inline const double* acc() const{return axyz;}

		private:
			void onImu(const sensor_msgs::Imu::ConstPtr& data);
			tf2_ros::Buffer buffer;
			tf2_ros::TransformListener listener;
			labust::math::unwrap unwrap;
			ros::Subscriber imu;
			double rpy[3], pqr[3], axyz[3];
		};

		class DvlHandler : public NewArrived
		{
		public:
			enum {u=0,v,w};

			DvlHandler():r(0),listener(buffer){};

			void configure(ros::NodeHandle& nh);

			inline const double* body_speeds() const {return uvw;};

			inline void current_r(double yaw_rate) {r = yaw_rate;};

		private:
			void onDvl(const geometry_msgs::TwistStamped::ConstPtr& data);
			double uvw[3], r;
			ros::Subscriber nu_dvl;
			tf2_ros::Buffer buffer;
			tf2_ros::TransformListener listener;
		};
	}
}
/* SENSORHANDLERS_HPP_ */
#endif
