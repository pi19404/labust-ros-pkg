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
#include <labust/control/HLControl.hpp>
#include "HeadingControl_test.hpp"
#include <labust/navigation/LFModel.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>

struct LFControl
{
	LFControl():Ts(0.1),LFKp(1){};

	void init()
	{
		ros::NodeHandle nh;
		refPoint = nh.subscribe<geometry_msgs::PointStamped>("LFPoint", 1,
				&LFControl::onNewPoint,this);

		initialize_controller();
	}

	void onNewPoint(const geometry_msgs::PointStamped::ConstPtr& point)
	{
		using labust::navigation::LFModel;
		Tt(0) = point->point.x;
		Tt(1) = point->point.y;
		Tt(2) = point->point.z;

		line.setLine(T0,Tt);
	};

	void step(const auv_msgs::NavSts::ConstPtr& state, std_msgs::Float32::Ptr& headingRef)
	{
		T0(0) = state->position.north;
		T0(1) = state->position.east;
		T0(2) = state->position.depth;

	  double dh = line.calculatedH(T0(0),T0(1),T0(2));
	  double d=sin(line.gamma())*(Tt(0)-T0(0))-cos(line.gamma())*(Tt(1)-T0(1));
	  double z=labust::math::coerce(LFKp*dh,-0.7,0.7);
	  std::cout<<"Distance to line:"<<d<<","<<dh<<", correction:"<<z<<","<<asin(z)<<std::endl;
	  headingRef->data = labust::math::wrapRad(line.gamma() + asin(z));
	}

	void initialize_controller()
	{
		ROS_INFO("Initializing lf-heading controller...");

		ros::NodeHandle nh;
		nh.param("lf_controller/heading_LFKp", LFKp,LFKp);
		nh.param("heading_control/sampling",Ts,Ts);

		ROS_INFO("Heading lf-heading controller initialized.");
	}

private:
	labust::navigation::LFModel::vector T0,Tt;
	labust::navigation::LFModel::Line line;
	ros::Subscriber refPoint;
	double Ts, LFKp;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"lf_control_test");
	//Initialize
	labust::control::HLControl<LFControl,
		labust::control::EnablePolicy,
		labust::control::NoWindup,
		std_msgs::Float32> controller;
	//Start execution.
	ros::spin();
	return 0;
}



