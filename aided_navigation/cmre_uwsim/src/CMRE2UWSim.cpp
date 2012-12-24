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
#include <std_msgs/Float64.h>

#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Matrix>

#include <boost/bind.hpp>

#include <stdlib.h>
#include <string.h>
#include <map>

void onUpdate(ros::Publisher* pub, labust::vehicles::stateMapRef state)
{
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

void onNewState(ros::Publisher* pub, const std_msgs::String::ConstPtr& msg)
{
	labust::xml::GyrosReader reader(msg->data);
	labust::vehicles::stateMap state;
	reader.dictionary(state);

	onUpdate(pub,state);
}

struct MoosPos
{
  MoosPos():
	nav_x("nav_x"),
	nav_y("nav_y"),
	nav_heading("nav_heading"){};

  typedef boost::shared_ptr<MoosPos> Ptr;
  std::map<std::string, double> pos;
  std::string nav_x, nav_y, nav_heading;
  std::map<std::string, bool> updatedPos;
};

void onNewState(std::string name,MoosPos::Ptr mpos, ros::Publisher* pub, const std_msgs::Float64::ConstPtr& msg)
{
	mpos->pos[name] = msg->data;
	mpos->updatedPos[name] = 1;

	bool flag = 1;

	for(std::map<std::string,bool>::iterator it=mpos->updatedPos.begin(); it != mpos->updatedPos.end(); ++it)
	{
	  flag = flag && it->second;
	}

	if (flag)
	{
		labust::vehicles::stateMap state;
		using namespace labust::vehicles::state;
		state[x]=mpos->pos[mpos->nav_x];
		state[y]=mpos->pos[mpos->nav_y];
		state[yaw]=mpos->pos[mpos->nav_heading]/180*M_PI;
		
		for(std::map<std::string,bool>::iterator it=mpos->updatedPos.begin(); it != mpos->updatedPos.end(); ++it)
		{
		  it->second=false;
		}

		onUpdate(pub,state);
	}
}

void createMOOSSubscription(ros::Publisher* pub)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  MoosPos::Ptr mpos(new MoosPos());
  pnh.param("nav_x",mpos->nav_x,mpos->nav_x);
  pnh.param("nav_y",mpos->nav_y,mpos->nav_y);
  pnh.param("nav_heading",mpos->nav_heading,mpos->nav_heading);

  ros::Subscriber uvstate1=nh.subscribe<std_msgs::Float64>(mpos->nav_x, 10, boost::bind(&onNewState,mpos->nav_x,mpos,pub,_1));
  ros::Subscriber uvstate2=nh.subscribe<std_msgs::Float64>(mpos->nav_y, 10, boost::bind(&onNewState,mpos->nav_y,mpos,pub,_1));
  ros::Subscriber uvstate3=nh.subscribe<std_msgs::Float64>(mpos->nav_heading, 10, boost::bind(&onNewState,mpos->nav_heading,mpos,pub,_1));

  ros::spin();
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "UVApp2UWSim");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string nav_type("xml"), uwsimName("dataNavigator");
	pnh.param("nav_type",nav_type,nav_type);
	pnh.param("OutputState",uwsimName,uwsimName);
	
	ros::Publisher position_pub;
	position_pub=nh.advertise<nav_msgs::Odometry>(uwsimName,10);

	if (nav_type == "moos")
	{
	  createMOOSSubscription(&position_pub);
	}
	else
	{
	    std::string stateName("uuv_state");
		pnh.param("InputState",stateName,stateName);
	  	ros::Subscriber uvstate=nh.subscribe<std_msgs::String>(stateName, 10, boost::bind(&onNewState,&position_pub,_1));
    	ros::spin();
	}

	return 0;
}




