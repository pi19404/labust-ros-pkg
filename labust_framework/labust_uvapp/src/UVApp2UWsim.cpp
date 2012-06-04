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
#include <labust/vehicles/vehiclesfwd.hpp>
#include <labust/xml/GyrosReader.hpp>



//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <MOOS_msgs/MOOSString.h>

#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Matrix>

#include <boost/bind.hpp>

#include <stdlib.h>
#include <string.h>

void onNewState(ros::Publisher* pub, const MOOS_msgs::MOOSString::ConstPtr& msg)
{
	labust::xml::GyrosReader reader(msg->msg);
	labust::vehicles::stateMap state;
	reader.dictionary(state);

	nav_msgs::Odometry odom;
	using namespace labust::vehicles::state;

	osg::Matrixd T, Rx, Ry, Rz, transform;
	T.makeTranslate(state[x],state[y],state[z]);
	Rx.makeRotate(state[roll],1,0,0);
	Ry.makeRotate(state[pitch],0,1,0);
	Rz.makeRotate(state[yaw],0,0,1);
	transform=Rz*Ry*Rx*T;
	osg::Vec3d trans=transform.getTrans();
	osg::Quat rot=transform.getRotate();

	odom.pose.pose.position.x=trans.x();
	odom.pose.pose.position.y=trans.y();
	odom.pose.pose.position.z=trans.z();
	odom.pose.pose.orientation.x=rot.x();
	odom.pose.pose.orientation.y=rot.y();
	odom.pose.pose.orientation.z=rot.z();
	odom.pose.pose.orientation.w=rot.w();

	odom.twist.twist.linear.x=0;
	odom.twist.twist.linear.y=0;
	odom.twist.twist.linear.z=0;
	odom.twist.twist.angular.x=0;
	odom.twist.twist.angular.y=0;
	odom.twist.twist.angular.z=0;
	for (int i=0; i<36; i++) {
		odom.twist.covariance[i]=0;
		odom.pose.covariance[i]=0;
	}
	pub->publish(odom);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "UVApp2UWSim");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

  std::string stateName("state"), uwsimName("dataNavigator");
	pnh.param("InputState",stateName,stateName);
	pnh.param("OutputState",uwsimName,uwsimName);

	ros::Publisher position_pub;
	position_pub=nh.advertise<nav_msgs::Odometry>(uwsimName,10);

	ros::Subscriber uvstate=nh.subscribe<MOOS_msgs::MOOSString>(stateName, 10, boost::bind(&onNewState,&position_pub,_1));

	ros::spin();

	return 0;
}




