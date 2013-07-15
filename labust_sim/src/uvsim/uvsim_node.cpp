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
#include <labust/ros/SimCore.hpp>
#include <labust/ros/SimSensors.hpp>
#include <labust/tools/conversions.hpp>

#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

/*struct sim_imu
{
	sim_imu(ros::NodelHandle& nh)
	{

	}

	void operator()(sensor_msgs::Imu::Ptr& imu,
			const labust::simulation::RBModel& model,
			tf::TransformBroadcaster& broadcaster,
			tf::TransformListener& listener)
	{
		using namespace labust::simulation;
		using namespace Eigen;

		imu->header.stamp = ros::Time::now();
		imu->header.frame_id = "imu_frame";
		labust::tools::vectorToPoint(model.NuAcc(), imu->linear_acceleration);
		labust::tools::vectorToPoint(&model.Nu()[RBModel::p], imu->angular_velocity);
		Quaternion<double> quat;
		labust::tools::quaternionFromEulerZYX(model.Eta()(RBModel::phi),
				model.Eta()(RBModel::theta),
				model.Eta()(RBModel::psi), quat);

		imu->orientation.x = quat.x();
		imu->orientation.y = quat.y();
		imu->orientation.z = quat.z();
		imu->orientation.w = quat.w();

		tf::Transform transform;
		transform.setOrigin(tf::Vector3(0, 0, 0));
		transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
		broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu_frame"));
	}
};

struct sim_gps
{
	void operator()(sensor_msgs::NavSatFix::Ptr& fix,
			const labust::simulation::RBModel& model,
			tf::TransformBroadcaster& broadcaster,
			tf::TransformListener& listener)
	{
		using namespace labust::simulation;
		using namespace Eigen;

		tf::StampedTransform transformLocal, transformDeg;

		try
		{
			//In case the origin changes
			listener.lookupTransform("/worldLatLon", "/world", ros::Time(0), transformDeg);
			originLat = transformDeg.getOrigin().y();
			originLon = transformDeg.getOrigin().x();
		}
		catch (tf::TransformException& ex)
		{
		   ROS_ERROR("%s",ex.what());
		}

		vector& eta = model.Eta();

		try
		{
		    listener.lookupTransform("base_link", "gps_frame", ros::Time(0), transformLocal);

		  	fix->altitude = eta(RBModel::z) - transformLocal.getOrigin().z() ;
		  	//gps_common::UTMtoLL(transform.getOrigin().y(), transform.getOrigin().x(), utmzone, fix->latitude, fix->longitude);
		  	std::pair<double, double> diffAngle = labust::tools::meter2deg(eta(VehicleModel6DOF::x),
		  		eta(VehicleModel6DOF::y),
		  		//The latitude angle
		  		originLat);

		  	modelLat = fix->latitude = originLat + diffAngle.first;
		  	modelLon = fix->longitude = originLon + diffAngle.second;
		    fix->header.stamp = ros::Time::now();
		    fix->header.frame_id = "worldLatLon";

				tf::Transform transform;
				Eigen::Quaternion<float> q;
				transform.setOrigin(tf::Vector3(eta(VehicleModel6DOF::x),
						eta(VehicleModel6DOF::y),
						eta(VehicleModel6DOF::z)));
				labust::tools::quaternionFromEulerZYX(eta(VehicleModel6DOF::phi),
						eta(VehicleModel6DOF::theta),
						eta(VehicleModel6DOF::psi), q);
				transform.setRotation(tf::Quaternion(q.x(),q.y(),q.z(),q.w()));
				gpsBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local", "base_link_noisy"));
		}
		catch (tf::TransformException& ex)
		{
		   ROS_ERROR("%s",ex.what());
		}

		return fix;
	}
};
*/
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"uvsim");
	labust::simulation::SimCore simulator;

	ros::NodeHandle nh;
	//labust::simulation::BasicSensor<sensor_msgs::Imu, sim_imu> imu(nh,"imu");

	ros::spin();
	return 0;
}
