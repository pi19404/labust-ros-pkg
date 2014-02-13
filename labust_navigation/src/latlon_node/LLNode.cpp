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
#include <labust/tools/conversions.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

struct LLNode
{
	LLNode():
		originLat(0),
		originLon(0),
		fixValidated(false),
		fixCount(0)
	{
		ros::NodeHandle nh,ph("~");
		nh.param("LocalOriginLat",originLat,originLat);
		nh.param("LocalOriginLon",originLon,originLon);
		ph.param("LocalFixSim",fixValidated, fixValidated);

		gps_raw = nh.subscribe<sensor_msgs::NavSatFix>("gps",1,&LLNode::onGps, this);

		runner = boost::thread(boost::bind(&LLNode::publishFrame, this));
	}

	~LLNode()
	{
		runner.join();
	}

	void onGps(const sensor_msgs::NavSatFix::ConstPtr& fix)
	{
		//In case we didn't have a fix on launch, but now we got 10 fixes in 15 sec.
		if (!fixValidated)
		{
			originLat = fix->latitude;
			originLon = fix->longitude;
			fixValidated = true;
		}
	};

	void publishFrame()
	{
		ros::Rate rate(1);
		while (ros::ok())
		{
			geometry_msgs::TransformStamped transform;
			if (fixValidated)
			{
				geometry_msgs::TransformStamped transform;
				transform.transform.translation.x = originLon;
				transform.transform.translation.y = originLat;
				transform.transform.translation.z = 0;
				labust::tools::quaternionFromEulerZYX(0, 0, 0,
						transform.transform.rotation);
				transform.child_frame_id = "world";
				transform.header.frame_id = "worldLatLon";
				transform.header.stamp = ros::Time::now();
				broadcaster.sendTransform(transform);
			}

			transform.transform.translation.x = 0;
			transform.transform.translation.y = 0;
			transform.transform.translation.z = 0;
			Eigen::Quaternion<float> q;
			labust::tools::quaternionFromEulerZYX(M_PI,0,M_PI/2,transform.transform.rotation);
			transform.child_frame_id = "local";
			transform.header.frame_id = "world";
			transform.header.stamp = ros::Time::now();
			broadcaster.sendTransform(transform);

			rate.sleep();
		}
	}

private:
	ros::Subscriber gps_raw;
	tf2_ros::TransformBroadcaster broadcaster;
	double originLat, originLon;
	bool fixValidated;
	int fixCount;
	boost::thread runner;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"llnode");
	ros::NodeHandle nh;
	LLNode llnode;
	ros::spin();
	return 0;
}


