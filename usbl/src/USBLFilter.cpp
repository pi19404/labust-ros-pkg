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
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

PLUGINLIB_DECLARE_CLASS(usbl,USBLFilter,labust::tritech::USBLFilter, nodelet::Nodelet)

using namespace labust::tritech;

USBLFilter::USBLFilter(){};

USBLFilter::~USBLFilter(){};

void USBLFilter::onInit()
{
	ros::NodeHandle nh = this->getNodeHandle();
	usblSub = nh.subscribe<geometry_msgs::PointStamped>("usblMeas", 1, boost::bind(&USBLFilter::onUsbl,this,_1));
	navPub = nh.advertise<nav_msgs::Odometry>("usblFiltered",1);

	worker = boost::thread(boost::bind(&USBLFilter::run,this));
}

void USBLFilter::onUsbl(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	NODELET_DEBUG("New update: %f %f %f\n",msg->point.x, msg->point.y, msg->point.z);

	boost::mutex::scoped_lock lock(dataMux);

  tf::StampedTransform transform;
  try
  {
      listener.lookupTransform("local", "usbl", ros::Time(0), transform);
  }
  catch (tf::TransformException& ex)
  {
     NODELET_ERROR("%s",ex.what());
  }

	KFilter::input_type vec(2);
	vec(0) = transform.getOrigin().x() + msg->point.x;
	vec(1) = transform.getOrigin().y() + msg->point.y;
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
		nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry());
		odom->header.stamp = ros::Time::now();
		odom->header.frame_id = "local";
		odom->child_frame_id = "local";
		const KFilter::vector& state = filter.getState();
		odom->pose.pose.position.x = state(KFilter::xp);
		odom->pose.pose.position.y = state(KFilter::yp);
		odom->pose.pose.orientation.z = state(KFilter::psi);
		//\todo Add covariance data
		odom->twist.twist.linear.x = state(KFilter::Vv)*cos(state(KFilter::psi));
		odom->twist.twist.linear.y = state(KFilter::Vv)*sin(state(KFilter::psi));
		//\todo Add covariance data
		navPub.publish(odom);
		rate.sleep();
	}
}


