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
#include <labust/navigation/SensorHandlers.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/GeoUtilities.hpp>

#include <kdl/frames.hpp>

using namespace labust::navigation;

void GPSHandler::configure(ros::NodeHandle& nh)
{
	gps = nh.subscribe<sensor_msgs::NavSatFix>("gps", 1,
			&GPSHandler::onGps, this);
}

void GPSHandler::onGps(const sensor_msgs::NavSatFix::ConstPtr& data)
{
	//Calculate to X-Y tangent plane
	tf::StampedTransform transformDeg, transformLocal;
	try
	{
		listener.lookupTransform("local", "gps_frame", ros::Time(0), transformLocal);
		listener.lookupTransform("/worldLatLon", "local", ros::Time(0), transformDeg);
		posxy =	labust::tools::deg2meter(data->latitude - transformDeg.getOrigin().y(),
					data->longitude - transformDeg.getOrigin().x(),
					transformDeg.getOrigin().y());

		originLL.first = transformDeg.getOrigin().y();
		originLL.second = transformDeg.getOrigin().x();

		posLL.first = data->latitude;
		posLL.second = data->longitude;

		isNew = true;
	}
	catch(tf::TransformException& ex)
	{
		ROS_WARN("Unable to decode GPS measurement. Missing frame : %s",ex.what());
	}
};

void ImuHandler::configure(ros::NodeHandle& nh)
{
	imu = nh.subscribe<sensor_msgs::Imu>("imu", 1,
			&ImuHandler::onImu, this);
}

void ImuHandler::onImu(const sensor_msgs::Imu::ConstPtr& data)
{
	tf::StampedTransform transform;
	try
	{
		listener.lookupTransform("base_link", "imu_frame", ros::Time(0), transform);
		tf::Quaternion meas(data->orientation.x,data->orientation.y,
				data->orientation.z,data->orientation.w);
		tf::Quaternion result = meas*transform.getRotation();
		KDL::Rotation::Quaternion(result.x(),result.y(),result.z(),result.w()).GetEulerZYX
				(rpy[yaw],rpy[pitch],rpy[roll]);
		rpy[yaw] = unwrap(rpy[yaw]);

		//Transform angular velocities

		pqr[p] = data->angular_velocity.x;
		pqr[q] = data->angular_velocity.y;
		pqr[r] = data->angular_velocity.z;

		//Transform the accelerations

		axyz[ax] = data->linear_acceleration.x;
		axyz[ay] = data->linear_acceleration.y;
		axyz[az] = data->linear_acceleration.z;

		isNew = true;
	}
	catch (tf::TransformException& ex)
	{
		ROS_WARN("Failed converting the IMU data: %s",ex.what());
	}
};

void DvlHandler::configure(ros::NodeHandle& nh)
{
	nu_dvl = nh.subscribe<geometry_msgs::TwistStamped>("dvl", 1,
			&DvlHandler::onDvl, this);
}

void DvlHandler::onDvl(const geometry_msgs::TwistStamped::ConstPtr& data)
{
	if (data->header.frame_id == "base_link")
	{
		uvw[u] = data->twist.linear.x;
		uvw[v] = data->twist.linear.y;
		uvw[w] = data->twist.linear.z;

		isNew = true;
	}
	else if (data->header.frame_id == "local")
	{
		tf::StampedTransform transform;
		tf::Vector3 meas(data->twist.linear.x,
				data->twist.linear.y,
				data->twist.linear.z);
		listener.lookupTransform("local", "base_link", ros::Time(0), transform);
		tf::Vector3 result = transform.getBasis()*meas;
		uvw[u] = result.x();
		uvw[v] = result.y();
		uvw[w] = result.z();

		isNew = true;
	}
}
