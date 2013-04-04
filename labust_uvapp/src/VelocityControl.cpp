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
#include <labust/control/VelocityControl.hpp>
#include <labust/tools/rosutils.hpp>

#include <auv_msgs/BodyForceReq.h>
#include <std_msgs/Byte.h>

#include <boost/bind.hpp>

#include <Eigen/Dense>

#include <cmath>

using labust::control::VelocityControl;

VelocityControl::VelocityControl():
	windupNote(false),
	runFlag(false),
	nh(ros::NodeHandle()),
	ph(ros::NodeHandle("~")),
	lastTime(ros::Time::now()){this->onInit();}

void VelocityControl::onInit()
{
	std::string name;

	//Initialize publishers
	tauOut = nh.advertise<auv_msgs::BodyForceReq>("tauOut", 1);
	//Windup flag
	windup = nh.advertise<std_msgs::Byte>("vcWindupFlag",1);

	//Initialze subscribers
	velocityRef = nh.subscribe<auv_msgs::BodyVelocityReq>("nuRef", 1,
			&VelocityControl::handleReference,this);
	stateHat = nh.subscribe<auv_msgs::NavSts>("stateHat", 1,
			&VelocityControl::handleEstimates,this);
	stateMeas = nh.subscribe<auv_msgs::NavSts>("stateMeas", 1,
			&VelocityControl::handleMeasurements,this);
	tauAch = nh.subscribe<auv_msgs::BodyForceReq>("tauAch", 1,
				&VelocityControl::handleWindup,this);

	nh.param("velocity_controller/synced",synced,true);

	initialize_controller();
}

void VelocityControl::handleReference(const auv_msgs::BodyVelocityReq::ConstPtr& ref)
{
	newReference = true;

	//Copy into controller
	controller[u].desired = ref->twist.linear.x;
	controller[v].desired = ref->twist.linear.y;
	controller[w].desired = ref->twist.linear.z;
	controller[p].desired = ref->twist.angular.x;
	controller[q].desired = ref->twist.angular.y;
	controller[r].desired = ref->twist.angular.z;

	disable_axis[u] = ref->disable_axis.x;
	disable_axis[v] = ref->disable_axis.y;
	disable_axis[w] = ref->disable_axis.z;
	disable_axis[p] = ref->disable_axis.roll;
	disable_axis[q] = ref->disable_axis.pitch;
	disable_axis[r] = ref->disable_axis.yaw;

	if (!synced && newEstimate) step();
}

void VelocityControl::handleMeasurements(const auv_msgs::NavSts::ConstPtr& ref)
{
	//Copy into identification controller
};

void VelocityControl::handleWindup(const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	//Quick testing addition
	//\todo Add per controller windup detection
	windupNote = true;
	controller[u].windup = 1;
	controller[u].tracking = tau->wrench.force.x;
	controller[v].windup = 1;
	controller[v].tracking = tau->wrench.force.y;
	controller[r].windup = 1;
	controller[r].tracking = tau->wrench.torque.z;
};

void VelocityControl::handleEstimates(const auv_msgs::NavSts::ConstPtr& estimate)
{
	newEstimate = true;

	//Copy into controller
	controller[u].state = estimate->body_velocity.x;
	controller[v].state = estimate->body_velocity.y;
	controller[w].state = estimate->body_velocity.z;
	controller[p].state = estimate->orientation_rate.roll;
	controller[q].state = estimate->orientation_rate.pitch;
	controller[r].state = estimate->orientation_rate.yaw;

	if (!synced && newReference) step();
};

void VelocityControl::step()
{
	auv_msgs::BodyForceReq tau;
	std_msgs::Byte windupFlag;

	tau.header.stamp = ros::Time::now();

	if (newReference && newEstimate)
	{
		float Ts = (ros::Time::now() - lastTime).toSec();
		if (Ts > 0.3) Ts = 0.1;
		lastTime = ros::Time::now();

		Ts = 0.1;

		ROS_INFO("VelocityControl::Sampling Time=%f",Ts);

		std::vector<float> scaling(r+2);

		for (int i=u; i<=r;++i)
		{
			if (disable_axis[i])
				controller[i].output = 0;
			else
			{
				PIFFController_step(&controller[i], Ts);
				if (controller[i].windup) windupFlag.data = windupFlag.data | (1<<i);
			}

			//scaling[i] = fabs(controller[i].output/controller[i].outputLimit);
			//if (scaling[i] != scaling[i]) scaling[i] = 0;

			std::cout<<i<<". has scaling:"<<scaling[i]<<","<<controller[i].output<<","<<controller[i].outputLimit<<std::endl;
		}
//
//		////////////////////////////Allocation stuff/////////////////////////////////////////
//		Eigen::Matrix<float, 3,4> B;
//		float cp(cos(M_PI/4)),sp(sin(M_PI/4));
//		B<<cp,cp,-cp,-cp,
//			 sp,-sp,sp,-sp,
//			 1,-1,-1,1;
//		Eigen::Matrix<float, 4,3> pinv = B.transpose()*(B*B.transpose()).inverse();
//
//		Eigen::Vector3f virtualInput;
//		virtualInput<<controller[u].output, controller[v].output, controller[r].output;
//
//		Eigen::Vector4f tdes = pinv*virtualInput;
//
//		enum{T1 = 0,T2,T3,T4};
//
//		float taumax(13/(2*cp));
//
//		Eigen::Vector4f scaled = tdes/taumax;
//		Eigen::Matrix<float,5,1> tscale;
//		tscale<<fabs(scaled(0)),fabs(scaled(1)),fabs(scaled(2)),fabs(scaled(3)),1.0f;
//		//std::for_each(tscale.data(), tscale.data() + tscale.SizeAtCompileTime, fabs);
//		std::sort(tscale.data(),tscale.data() + tscale.SizeAtCompileTime);
//
//		std::cout<<"Sorted scales:";
//		for (int i=0; i<tscale.SizeAtCompileTime; ++i)
//		{
//			std::cout<<tscale(i)<<",";
//		}
//		std::cout<<std::endl;
//
//		//With scaling
//		tdes = tdes/tscale(tscale.SizeMinusOne);
//		//Without scaling
//		for (int i=0; i<4; ++i)
//		{
//			//if (fabs(tdes(i))>taumax) tdes(i) *= taumax/fabs(tdes(i));
//		}
//
//		Eigen::Vector3f virtualInputLim = B*tdes;
//		/////////////////////////////////////////////////////////////////////////////////////
//		float scale = tscale(tscale.SizeMinusOne);
//		std::cout<<"Take scaling:"<<tscale(tscale.SizeMinusOne)<<std::endl;
//
//		std::cout<<std::endl;
//
//		std::cout<<"Virtual input:";
//		for (int i=0;i<3;++i) std::cout<<virtualInput(i)<<",";
//		std::cout<<std::endl;
//		std::cout<<"Desired u:";
//		for (int i=0;i<4;++i) std::cout<<tdes(i)<<",";
//		std::cout<<std::endl;
//		std::cout<<"Virtual input limited:";
//		for (int i=0;i<3;++i) std::cout<<virtualInputLim(i)<<",";
//		std::cout<<std::endl;
//
//		ROS_INFO("Thrust:%f,%f,%f,%f",tdes(0),tdes(1),tdes(2),tdes(3));
//
//
//		std::cout<<std::endl;
//
//		Eigen::Matrix<float,6,1> vin;
//		vin<<virtualInputLim(0),virtualInputLim(1),0,0,0,virtualInputLim(2);
//		//windupFlag.data = 0;
//		for (int i=u; i<=r; ++i)
//		{
//			if (controller[i].autoTracking == 0 && (!disable_axis[i]))
//			{
//				controller[i].tracking = vin(i);
//				controller[i].windup = scale>1;
//				controller[i].output = vin(i);
//				//controller[i].integratorState = vin(i);
//				std::cout<<"Doing external tracking."<<std::endl;
//				//controller[i].tracking = controller[i].output;
//				//PIDController_trackingUpdate(&controller[i],Ts,1);
//				std::cout<<i<<"Output after scaling:"<<controller[i].output<<std::endl;
//			}
//			else
//			{
//				//controller[i].windup = 0;
//				//std::cout<<"Acting stupid as shit:"<<int(controller[i].autoTracking)<<std::endl;
//			}
//
//			//if (controller[i].windup)
//		  	//windupFlag.data = windupFlag.data | (1<<i);
//		}

 		//Copy to tau
		tau.wrench.force.x = controller[u].output;
		tau.wrench.force.y = controller[v].output;
		tau.wrench.force.z = controller[w].output;
		tau.wrench.torque.x = controller[p].output;
		tau.wrench.torque.y = controller[q].output;
		tau.wrench.torque.z = controller[r].output;

		//Restart values
		windupNote = newReference = newEstimate = false;
		tauOut.publish(tau);
	}
	else
	{
		ROS_WARN("VelocityControl - messages are out of sync.");
	}

	//tauOut.publish(tau);
	windup.publish(windupFlag);
}

void VelocityControl::start()
{
	ros::Rate rate(10);

	while (ros::ok())
	{
		if (synced)
		{
			ros::spinOnce();
			step();
			rate.sleep();
		}
		else
			ros::spin();
	}
}

void VelocityControl::initialize_controller()
{
	ROS_INFO("Initializing velocity controller...");

	Eigen::Vector6d closedLoopFreq(Eigen::Vector6d::Zero());
	labust::tools::getMatrixParam(nh,"velocity_controller/closed_loop_freq", closedLoopFreq);
	Eigen::Vector6d outputLimit(Eigen::Vector6d::Zero());
	labust::tools::getMatrixParam(nh,"velocity_controller/output_limits", outputLimit);

	labust::tools::DynamicsModel model(nh);
	Eigen::Vector6d alphas(model.added_mass);
	Eigen::Vector6d alpha_mass;
	alpha_mass<<model.mass,model.mass,model.mass,
			model.inertia_matrix.diagonal();
	alphas += alpha_mass;

	bool autoTracking(0);
	nh.param("velocity_controller/auto_tracking",autoTracking,autoTracking);

	for (int32_t i = u; i <r; ++i)
	{
		PIDController_init(&controller[i]);
		controller[i].closedLoopFreq = closedLoopFreq(i);
		controller[i].outputLimit = outputLimit(i);
		controller[i].modelParams[alpha] = alphas(i);
		controller[i].modelParams[beta] = model.damping(i);
		controller[i].modelParams[betaa] = model.qdamping(i);

		PIFFController_tune(&controller[i]);
		controller[i].autoTracking = autoTracking;

		ROS_INFO("Controller %d:",i);
		ROS_INFO("ModelParams: %f %f %f",controller[i].modelParams[alpha], controller[i].modelParams[beta],
				controller[i].modelParams[betaa]);
		ROS_INFO("Gains: %f %f %f",controller[i].gains[Kp], controller[i].gains[Ki],
				controller[i].gains[Kt]);
	}

	ROS_INFO("Velocity controller initialized.");
}
