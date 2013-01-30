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
 *  Author : Dula Nad
 *  Created: 23.01.2013.
 *********************************************************************/
#include <labust/control/PIDController.hpp>

#include <ros/ros.h>
#include <auv_msgs/BodyForceReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/NavSts.h>

bool newRef=false, newState=false;

void handleRefIn(auv_msgs::BodyVelocityReq* nuRef, const auv_msgs::BodyVelocityReq::ConstPtr msg)
{
	*nuRef = *msg;
	newRef = true;
}

void handleNuIn(auv_msgs::NavSts* nu, const auv_msgs::NavSts::ConstPtr msg)
{
	*nu = *msg;
	newState = true;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"velocity_demo");

	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

	auv_msgs::BodyVelocityReq nuRef;
	auv_msgs::NavSts nu;

	//Publishers
	ros::Publisher tauOut = nh.advertise<auv_msgs::BodyForceReq>("tauIn",10);
	//Subscribers
	ros::Subscriber velocityRefIn = nh.subscribe<auv_msgs::BodyVelocityReq>("nuRef", 10, boost::bind(&handleRefIn,&nuRef,_1));
	ros::Subscriber velocityIn = nh.subscribe<auv_msgs::NavSts>("stateHat", 10, boost::bind(&handleNuIn,&nu,_1));

	ros::Rate rate(10);

	labust::control::PID pid(1,0,0);

	while (ros::ok())
	{
		if (newRef && newState)
		{
			auv_msgs::BodyForceReq tau;
			//algoritam mjenja tau
			tau.wrench.force.x = pid.step(nuRef.twist.linear.x,nu.body_velocity.x);
			tauOut.publish(tau);

			newRef = false;
			newState = false;
		}
		else
		{
			std::cerr<<"No new messages"<<std::endl;
			auv_msgs::BodyForceReq tau;
			tauOut.publish(tau);
		}

		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}



