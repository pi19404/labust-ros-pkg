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
#include <labust/control/VelocityControl.hpp>

#include <auv_msgs/BodyForceReq.h>

#include <boost/bind.hpp>

using labust::control::VelocityControl;

VelocityControl::VelocityControl():
	runFlag(false),
	nh(),
	ph("~")
{
	std::string name;

	//Initialize publishers
	name = "tauOut";
	ph.param("TauOut", name, name);
	tauOut = nh.advertise<auv_msgs::BodyForceReq>(name, 1);

	//Initialze subscribers
	name = "nuRef";	ph.param("VelocityReference", name, name);
	velocityRef = nh.subscribe<auv_msgs::BodyVelocityReq>(name, 1,
			&VelocityControl::handleReference,this);

	name = "stateHat"; ph.param("StateEstimates", name, name);
	stateHat = nh.subscribe<auv_msgs::NavSts>(name, 1,
			&VelocityControl::handleEstimates,this);

	name = "stateMeas"; ph.param("StateMeasurements", name, name);
	stateMeas = nh.subscribe<auv_msgs::NavSts>(name, 1,
			&VelocityControl::handleMeasurements,this);

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
}

void VelocityControl::handleMeasurements(const auv_msgs::NavSts::ConstPtr& ref)
{
	//Copy into identification controller
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
};

void VelocityControl::start()
{
	ros::Rate rate(10);

	float Ts = 0.1;

	while (ros::ok())
	{
		auv_msgs::BodyForceReq tau;

		if (newReference && newEstimate)
		{

			for (int i=u; i<=r;++i)
			{
				if (disable_axis[i])
					controller[i].output = 0;
				else
					PIFFController_step(&controller[i], Ts);
			}

   		//Copy to tau
			tau.wrench.force.x = controller[u].output;
			tau.wrench.force.y = controller[v].output;
			tau.wrench.force.z = controller[w].output;
			tau.wrench.torque.x = controller[p].output;
			tau.wrench.torque.y = controller[q].output;
			tau.wrench.torque.z = controller[r].output;

			//Restart values
			newReference = newEstimate = false;
		}
		else
		{
			ROS_WARN("VelocityControl - messages are out of sync.");
		}

		tauOut.publish(tau);

		rate.sleep();
		ros::spinOnce();
	}
}

void VelocityControl::initialize_controller()
{
	ROS_INFO("Initializing velocity controller...");

	for (int i=u; i<=r;++i) PIDController_init(&controller[i]);

	//Get closed loop parameters
	XmlRpc::XmlRpcValue controllerParams;
	nh.getParam("velocity_controller/closed_loop_freq", controllerParams);
	ROS_ASSERT(controllerParams.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for (int32_t i = u; i < controllerParams.size(); ++i)
	{
	  ROS_ASSERT(controllerParams[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	  controller[i].closedLoopFreq = static_cast<double>(controllerParams[i]);
	}

	controllerParams.clear();
	nh.getParam("velocity_controller/output_limits", controllerParams);
	ROS_ASSERT(controllerParams.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for (int32_t i = u; i < controllerParams.size(); ++i)
	{
	  ROS_ASSERT(controllerParams[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	  controller[i].outputLimit = static_cast<double>(controllerParams[i]);
	}

	//Get model parameters
	std::string modelName("default");
	nh.param("model_name",modelName,modelName);

	//Inertia and added mass
	double mass(1);
	nh.param(modelName+"/dynamics/mass",mass,mass);
	controller[u].modelParams[alpha] = mass;
	controller[v].modelParams[alpha] = mass;
	controller[w].modelParams[alpha] = mass;

	XmlRpc::XmlRpcValue modelParams;
	nh.getParam(modelName+"/dynamics/inertia_matrix", modelParams);
	ROS_ASSERT(modelParams.getType() == XmlRpc::XmlRpcValue::TypeArray);

	controller[p].modelParams[alpha] = static_cast<double>(modelParams[0]);
	controller[q].modelParams[alpha] = static_cast<double>(modelParams[4]);
	controller[r].modelParams[alpha] = static_cast<double>(modelParams[8]);

	modelParams.clear();
	nh.getParam(modelName+"/dynamics/added_mass", modelParams);
	ROS_ASSERT(modelParams.getType() == XmlRpc::XmlRpcValue::TypeArray);
	for (int32_t i = u; i < modelParams.size(); ++i)
	{
	  ROS_ASSERT(modelParams[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	  controller[i].modelParams[alpha] += static_cast<double>(modelParams[i]);
	}

	//Linear damping
	modelParams.clear();
	nh.getParam(modelName+"/dynamics/damping", modelParams);
	ROS_ASSERT(modelParams.getType() == XmlRpc::XmlRpcValue::TypeArray);
	for (int32_t i = u; i < modelParams.size(); ++i)
	{
	  ROS_ASSERT(modelParams[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	  controller[i].modelParams[beta] = static_cast<double>(modelParams[i]);
	}
	//Quadratic damping
	modelParams.clear();
	nh.getParam(modelName+"/dynamics/quadratic_damping", modelParams);
	ROS_ASSERT(modelParams.getType() == XmlRpc::XmlRpcValue::TypeArray);
	for (int32_t i = u; i < modelParams.size(); ++i)
	{
	  ROS_ASSERT(modelParams[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	  controller[i].modelParams[betaa] = static_cast<double>(modelParams[i]);
	}

	for (int i=u; i<=r;++i)
	{
		PIFFController_tune(&controller[i]);

		ROS_INFO("Controller %d:",i);
		ROS_INFO("ModelParams: %f %f %f",controller[i].modelParams[alpha], controller[i].modelParams[beta],
				controller[i].modelParams[betaa]);
		ROS_INFO("Gains: %f %f %f",controller[i].gains[Kp], controller[i].gains[Ki],
				controller[i].gains[Kt]);

	}

  ROS_INFO("Velocity controller initialized.");
}
