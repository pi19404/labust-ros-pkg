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
 *  Created: 02.04.2013.
 *********************************************************************/
#include <labust/tritech/USBLFilter.hpp>
#include <labust/tools/GeoUtilities.hpp>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PointStamped.h>
#include <auv_msgs/NavSts.h>

PLUGINLIB_DECLARE_CLASS(usbl,USBLFilter,labust::tritech::USBLFilter, nodelet::Nodelet)

using namespace labust::tritech;

USBLFilter::USBLFilter(){};

USBLFilter::~USBLFilter(){};

void USBLFilter::onInit()
{
	ros::NodeHandle nh = this->getNodeHandle();
	usblSub = nh.subscribe<geometry_msgs::PointStamped>("usbl_nav", 1, boost::bind(&USBLFilter::onUsbl,this,_1));
	navPub = nh.advertise<auv_msgs::NavSts>("usblFiltered",1);

	worker = boost::thread(boost::bind(&USBLFilter::run,this));
}

void USBLFilter::onUsbl(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	NODELET_DEBUG("New update: %f %f %f\n",msg->point.x, msg->point.y, msg->point.z);

	boost::mutex::scoped_lock lock(dataMux);

	tf::StampedTransform transform;
	try
	{
		listener.lookupTransform("local", "base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException& ex)
	{
		NODELET_ERROR("%s",ex.what());
	}

	KFilter::input_type vec(2);
	vec(0) = transform.getOrigin().x() + msg->point.x;
	vec(1) = transform.getOrigin().y() + msg->point.y;
	depth = transform.getOrigin().z() + msg->point.z;
	filter.correct(vec);
};

void USBLFilter::run()
{
	///\todo Add variable rate of filter estimates.
	ros::Rate rate(10);

	filter.setTs(0.1);

	while (ros::ok())
	{
		boost::mutex::scoped_lock lock(dataMux);

		filter.predict();
		auv_msgs::NavSts::Ptr odom(new auv_msgs::NavSts());
		odom->header.stamp = ros::Time::now();
		odom->header.frame_id = "local";
		const KFilter::vector& state = filter.getState();
		odom->position.north = state(KFilter::xp);
		odom->position.east = state(KFilter::yp);
		odom->position.depth = depth;
		odom->orientation.yaw = state(KFilter::psi);
		//\todo Add covariance data
		odom->body_velocity.x = state(KFilter::Vv)*cos(state(KFilter::psi));

		try
		{
			tf::StampedTransform transformDeg;
			listener.lookupTransform("/worldLatLon", "local", ros::Time(0), transformDeg);

			std::pair<double, double> diffAngle = labust::tools::meter2deg(state(KFilter::xp),
					state(KFilter::yp),
					//The latitude angle
					transformDeg.getOrigin().y());
			odom->global_position.latitude = transformDeg.getOrigin().y() + diffAngle.first;
			odom->global_position.longitude = transformDeg.getOrigin().x() + diffAngle.second;
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
		}

		//\todo Add covariance data
		navPub.publish(odom);
		rate.sleep();
	}
}


