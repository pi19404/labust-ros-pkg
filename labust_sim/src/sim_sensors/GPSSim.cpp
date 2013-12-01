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
#include <labust/tools/GeoUtilities.hpp>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

struct GPSSim
{
	GPSSim():
		rate(10),
		gps_z(0)
	{
		ros::NodeHandle nh,ph("~");
		ph.param("gps_pub",rate,rate);
		ph.param("gps_height",gps_z,gps_z);

		odom = nh.subscribe<nav_msgs::Odometry>("meas_odom",1,&GPSSim::onOdom, this);
		gps_pub = nh.advertise<sensor_msgs::NavSatFix>("fix",1);
	}

	void onOdom(const typename nav_msgs::Odometry::ConstPtr& msg)
	{
		sensor_msgs::NavSatFix::Ptr fix(new sensor_msgs::NavSatFix());
		fix->header.stamp = ros::Time::now();
		fix->header.frame_id = "worldLatLon";

		//Broadcast the position of the GPS device.
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(0, 0, -gps_z));
		transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
		broadcaster.sendTransform(
				tf::StampedTransform(transform, ros::Time::now(),
						"base_link", "gps_frame"));

		tf::StampedTransform transformLocal, transformDeg;
		try
		{
			//Get the GPS position
			listener.lookupTransform("base_link", "gps_frame",
					ros::Time(0), transformLocal);
			double originLat = transformDeg.getOrigin().y();
			double originLon = transformDeg.getOrigin().x();
			//In case the origin changes
			listener.waitForTransform("/worldLatLon", "/world",
					ros::Time(0), ros::Duration(5.0));
			listener.lookupTransform("/worldLatLon", "/world",
					ros::Time(0), transformDeg);

			fix->altitude = -(transformLocal.getOrigin().z() + msg->pose.pose.position.z);
			std::pair<double, double> diffAngle =
					labust::tools::meter2deg(msg->pose.pose.position.x,
					msg->pose.pose.position.y,
					//The latitude angle
					originLat);

			fix->latitude = originLat + diffAngle.first;
			fix->longitude = originLon + diffAngle.second;

			static int i=0;
			i=++i%rate;
			if (fix->altitude>=-0.1 && (i == 0))
			{
				gps_pub.publish(fix);
			}
		}
		catch (tf::TransformException& ex)
		{
			ROS_WARN("%s",ex.what());
		}
	}

private:
	ros::Subscriber odom;
	ros::Publisher gps_pub;
	int rate;
	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;
	double gps_z;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"gps_sim");
	ros::NodeHandle nh;
	GPSSim gps;
	ros::spin();
	return 0;
}


