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
#include <labust/simulation/matrixfwd.hpp>
#include <labust/ros/SimSensors.hpp>
#include <labust/tools/conversions.hpp>
#include <labust/simulation/RBModel.hpp>
#include <labust/tools/GeoUtilities.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace labust
{
	namespace simulation
	{
		struct sim_pressure
		{
			void operator()(sensor_msgs::FluidPressure::Ptr& pressure,
					const SimSensorInterface::Hook& data)
			{
				using namespace labust::simulation;
				using namespace Eigen;

				pressure->header.stamp = ros::Time::now();
				pressure->header.frame_id = "local";

				const labust::simulation::vector& nu =
						data.noisy?data.model.NuNoisy():data.model.Nu();

				pressure->fluid_pressure = data.model.getPressure(nu(RBModel::z));
			}
		};

		struct sim_imu
		{
			void operator()(sensor_msgs::Imu::Ptr& imu,
					const SimSensorInterface::Hook& data)
			{
				using namespace labust::simulation;
				using namespace Eigen;

				imu->header.stamp = ros::Time::now();
				imu->header.frame_id = "imu_frame";
				const labust::simulation::vector& nu =
						data.noisy?data.model.NuNoisy():data.model.Nu();
				labust::tools::vectorToPoint(data.model.NuAcc(), imu->linear_acceleration);
				labust::tools::vectorToPoint(nu, imu->angular_velocity,3);

				const labust::simulation::vector& eta =
						data.noisy?data.model.EtaNoisy():data.model.Eta();
				Quaternion<double> quat;
				labust::tools::quaternionFromEulerZYX(eta(RBModel::phi),
						eta(RBModel::theta),
						eta(RBModel::psi), quat);

				imu->orientation.x = quat.x();
				imu->orientation.y = quat.y();
				imu->orientation.z = quat.z();
				imu->orientation.w = quat.w();

				tf::Transform transform;
				transform.setOrigin(tf::Vector3(0, 0, 0));
				transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
				data.broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu_frame"));
			}
		};

		///\todo Separate into a header and cpp file
		///\todo Add private handle for the configuration
		struct GPSSensor : public SimSensorInterface
		{
			GPSSensor():rate(10){};

			void configure(ros::NodeHandle& nh, const std::string& topic_name)
			{
				pub = nh.advertise<sensor_msgs::NavSatFix>(topic_name,1);
				ros::NodeHandle ph("~");
				ph.param("gps_pub",rate,rate);
			};

			void step(const Hook& data)
			{
				static int i=0;
				if ((i=++i%rate) == 0)
				{
					sensor_msgs::NavSatFixPtr msg(new sensor_msgs::NavSatFix());
					(*this)(msg, data);
					pub.publish(msg);
				}
			}

			void operator()(sensor_msgs::NavSatFix::Ptr& fix,
					const SimSensorInterface::Hook& data)
			{
				using namespace labust::simulation;
				using namespace Eigen;

				tf::Transform transform2;
				transform2.setOrigin(tf::Vector3(0, 0, 0));
				transform2.setRotation(tf::createQuaternionFromRPY(0,0,0));
				data.broadcaster.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "gps_frame"));

				tf::StampedTransform transformLocal, transformDeg;
				try
				{
					//In case the origin changes
					data.listener.lookupTransform("/worldLatLon", "/world", ros::Time(0), transformDeg);
					data.originLat = transformDeg.getOrigin().y();
					data.originLon = transformDeg.getOrigin().x();
				}
				catch (tf::TransformException& ex)
				{
					ROS_ERROR("%s",ex.what());
				}

				try
				{
					data.listener.lookupTransform("base_link", "gps_frame", ros::Time(0), transformLocal);

					const labust::simulation::vector& eta = (data.noisy ? data.model.EtaNoisy():data.model.Eta());
					fix->altitude = 0;
					fix->altitude = eta(RBModel::z) - transformLocal.getOrigin().z();
					std::pair<double, double> diffAngle = labust::tools::meter2deg(eta(RBModel::x),
							eta(RBModel::y),
							//The latitude angle
							data.originLat);

					fix->latitude = data.originLat + diffAngle.first;
					fix->longitude = data.originLon + diffAngle.second;
					fix->header.stamp = ros::Time::now();
					fix->header.frame_id = "worldLatLon";
//					if (fix->altitude < 0)
//					{
//						fix->status= fix->status.STATUS_NO_FIX;
//					}
//					else
//					{
//						fix->status = fix->status.STATUS_FIX;
//					}
				}
				catch (tf::TransformException& ex)
				{
					ROS_ERROR("%s",ex.what());
				}
			}

		private:
				ros::Publisher pub;
				int rate;
		};

		typedef BasicSensor<sensor_msgs::Imu, sim_imu> ImuSensor;
		typedef BasicSensor<sensor_msgs::FluidePressure, sim_pressure> PressureSensor;
		//typedef BasicSensor<sensor_msgs::NavSatFix, GPSSim> GPSSensor;
	}
}

PLUGINLIB_EXPORT_CLASS(labust::simulation::ImuSensor,
		labust::simulation::SimSensorInterface)
PLUGINLIB_EXPORT_CLASS(labust::simulation::GPSSensor,
		labust::simulation::SimSensorInterface)


