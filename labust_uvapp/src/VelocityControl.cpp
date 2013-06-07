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
#include <labust/math/NumberManipulation.hpp>

#include <auv_msgs/BodyForceReq.h>
#include <boost/bind.hpp>

#include <cmath>
#include <vector>
#include <string>

using labust::control::VelocityControl;

const std::string VelocityControl::dofName[]=
{"Surge","Sway","Heave","Roll","Pitch","Yaw"};

VelocityControl::VelocityControl():
			nh(ros::NodeHandle()),
			ph(ros::NodeHandle("~")),
			lastEst(ros::Time::now()),
			lastRef(ros::Time::now()),
			lastMan(ros::Time::now()),
			lastMeas(ros::Time::now()),
			timeout(0.5),
			joy_scale(1),
			Ts(0.1),
			server(serverMux)
{this->onInit();}

void VelocityControl::onInit()
{
	std::string name;
	//Initialize publishers
	tauOut = nh.advertise<auv_msgs::BodyForceReq>("tauOut", 1);
	tauAchW = nh.advertise<auv_msgs::BodyForceReq>("tauAchVelCon", 1);

	//Initialze subscribers
	velocityRef = nh.subscribe<auv_msgs::BodyVelocityReq>("nuRef", 1,
			&VelocityControl::handleReference,this);
	stateHat = nh.subscribe<auv_msgs::NavSts>("stateHat", 1,
			&VelocityControl::handleEstimates,this);
	measSub = nh.subscribe<auv_msgs::NavSts>("meas", 1,
			&VelocityControl::handleMeasurement,this);
	tauAch = nh.subscribe<auv_msgs::BodyForceReq>("tauAch", 1,
			&VelocityControl::handleWindup,this);
	manualIn = nh.subscribe<sensor_msgs::Joy>("joy",1,
			&VelocityControl::handleManual,this, ros::TransportHints().unreliable());
	//Configure service
	highLevelSelect = nh.advertiseService("ConfigureVelocityController",
			&VelocityControl::handleServerConfig, this);
	enableControl = nh.advertiseService("VelCon_enable",
			&VelocityControl::handleEnableControl, this);

	nh.param("velocity_controller/joy_scale",joy_scale,joy_scale);
	nh.param("velocity_controller/timeout",timeout,timeout);

	//Configure the dynamic reconfigure server
	server.setCallback(boost::bind(&VelocityControl::dynrec_cb, this, _1, _2));

	initialize_controller();
	config.__fromServer__(ph);
	server.setConfigDefault(config);
	this->updateDynRecConfig();
}

bool VelocityControl::handleEnableControl(labust_uvapp::EnableControl::Request& req,
		labust_uvapp::EnableControl::Response& resp)
{
	if (!req.enable)
	{
		for (int i=u; i<=r;++i) axis_control[i] = disableAxis;
		this->updateDynRecConfig();
	}

	return true;
}

void VelocityControl::updateDynRecConfig()
{
	ROS_INFO("Updating the dynamic reconfigure parameters. %d");

	config.Surge_mode = axis_control[u];
	config.Sway_mode = axis_control[v];
	config.Heave_mode = axis_control[w];
	config.Roll_mode = axis_control[p];
	config.Pitch_mode = axis_control[q];
	config.Yaw_mode = axis_control[r];

	config.High_level_controller="0 - None\n 1 - DP";

	server.updateConfig(config);
}

bool VelocityControl::handleServerConfig(labust_uvapp::ConfigureVelocityController::Request& req,
		labust_uvapp::ConfigureVelocityController::Response& resp)
{
	axis_control = req.desired_mode;
	this->updateDynRecConfig();
	return true;
}

void VelocityControl::handleReference(const auv_msgs::BodyVelocityReq::ConstPtr& ref)
{
	//Copy into controller
	controller[u].desired = ref->twist.linear.x;
	controller[v].desired = ref->twist.linear.y;
	controller[w].desired = ref->twist.linear.z;
	controller[p].desired = ref->twist.angular.x;
	controller[q].desired = ref->twist.angular.y;
	controller[r].desired = ref->twist.angular.z;

	lastRef = ros::Time::now();
	//newReference = true;
	//if (newEstimate) step();
}

void VelocityControl::handleManual(const sensor_msgs::Joy::ConstPtr& joy)
{
	tauManual[X] = config.Surge_joy_scale * joy->axes[1];
	tauManual[Y] = -config.Sway_joy_scale * joy->axes[0];
	tauManual[Z] = -config.Heave_joy_scale * joy->axes[3];
	tauManual[K] = 0;
	tauManual[M] = 0;
	tauManual[N] = -config.Yaw_joy_scale * joy->axes[2];
	lastMan = ros::Time::now();
}

void VelocityControl::dynrec_cb(labust_uvapp::VelConConfig& config, uint32_t level)
{
	this->config = config;

	config.__toServer__(ph);
	for(size_t i=u; i<=r; ++i)
	{
		int newMode(0);
		ph.getParam(dofName[i]+"_mode", newMode);
		//Stop the identification if it was aborted remotely.
		if ((axis_control[i] == identAxis) &&
				(newMode != identAxis) &&
				(ident[i] != 0)) ident[i].reset();

		axis_control[i] = newMode;
	}
}

void VelocityControl::handleWindup(const auv_msgs::BodyForceReq::ConstPtr& tau)
{
	if (!controller[u].autoTracking) controller[u].tracking = tau->wrench.force.x;
	if (!controller[v].autoTracking) controller[v].tracking = tau->wrench.force.y;
	if (!controller[w].autoTracking) controller[w].tracking = tau->wrench.force.z;
	if (!controller[p].autoTracking) controller[p].tracking = tau->wrench.torque.x;
	if (!controller[q].autoTracking) controller[q].tracking = tau->wrench.torque.y;
	if (!controller[r].autoTracking) controller[r].tracking = tau->wrench.torque.z;

	if (!controller[u].autoTracking) controller[u].windup = tau->disable_axis.x;
	if (!controller[v].autoTracking) controller[v].windup = tau->disable_axis.y;
	if (!controller[w].autoTracking) controller[w].windup = tau->disable_axis.z;
	if (!controller[p].autoTracking) controller[p].windup = tau->disable_axis.roll;
	if (!controller[q].autoTracking) controller[q].windup = tau->disable_axis.pitch;
	if (!controller[r].autoTracking) controller[r].windup = tau->disable_axis.yaw;
};

void VelocityControl::handleEstimates(const auv_msgs::NavSts::ConstPtr& estimate)
{
	//Copy into controller
	controller[u].state = estimate->body_velocity.x;
	controller[v].state = estimate->body_velocity.y;
	controller[w].state = estimate->body_velocity.z;
	controller[p].state = estimate->orientation_rate.roll;
	controller[q].state = estimate->orientation_rate.pitch;
	controller[r].state = estimate->orientation_rate.yaw;

	lastEst = ros::Time::now();
	//ROS_INFO("Semi-travel time:%f",(lastEst - estimate->header.stamp).toSec());
	//lastEst = estimate->header.stamp;
	//newEstimate = true;
	//if (newReference) step();
};

void VelocityControl::handleMeasurement(const auv_msgs::NavSts::ConstPtr& meas)
{
	//Copy into controller
	measurement[u] = meas->position.north;
	measurement[v] = meas->position.east;
	measurement[w] = meas->position.depth;
	measurement[p] = meas->orientation.roll;
	measurement[q] = meas->orientation.pitch;
	measurement[r] = meas->orientation.yaw;

	lastMeas = ros::Time::now();
	//newEstimate = true;
	//if (newReference) step();
};

void VelocityControl::safetyTest()
{
	bool refTimeout = (ros::Time::now() - lastRef).toSec() > timeout;
	bool estTimeout = (ros::Time::now() - lastEst).toSec() > timeout;
	bool manTimeout = (ros::Time::now() - lastMan).toSec() > timeout;
	bool measTimeout = (ros::Time::now() - lastMeas).toSec() > timeout;
	bool changed = false;

	for (int i=u; i<=r;++i)
	{
		bool cntChannel = (refTimeout || estTimeout) && (axis_control[i] == controlAxis);
		if (cntChannel) ROS_WARN("Timeout on the control channel. Controlled axes will be disabled.");
		bool measChannel = measTimeout && (axis_control[i] == identAxis);
		if (measChannel) ROS_WARN("Timeout on the measurement channel. Stopping identifications in progress.");
		bool manChannel = manTimeout && (axis_control[i] == manualAxis);
		if (manChannel) ROS_WARN("Timeout on the manual channel. Manual axes will be disabled.");

		suspend_axis[i] = (cntChannel || measChannel || manChannel);
	}

	//Update only on change.
	//if (changed) this->updateDynRecConfig();
}

double VelocityControl::doIdentification(int i)
{
	if (ident[i] == 0)
	{
		double C,X,ref;
		ph.getParam(dofName[i]+"_ident_amplitude",C);
		ph.getParam(dofName[i]+"_ident_hysteresis",X);
		ph.getParam(dofName[i]+"_ident_ref",ref);
		ident[i].reset(new SOIdentification());
		ident[i]->setRelay(C,X);
		ident[i]->Ref(ref);
		ROS_INFO("Started indentification of %s DOF.",dofName[i].c_str());
	}

	if (ident[i]->isFinished())
	{
		//Get parameters
		SOIdentification::ParameterContainer params;
		ident[i]->parameters(&params);
		controller[i].modelParams[alpha] = params[SOIdentification::alpha];
		controller[i].modelParams[beta] = params[SOIdentification::kx];
		controller[i].modelParams[betaa] = params[SOIdentification::kxx];
		//Tune controller
		PIFFController_tune(&controller[i]);
		//Write parameters to server
		XmlRpc::XmlRpcValue vparam;
		vparam.setSize(SOIdentification::numParams);
		vparam[0] = params[SOIdentification::alpha];
		vparam[1] = params[SOIdentification::kx];
		vparam[2] = params[SOIdentification::kxx];
		vparam[3] = params[SOIdentification::delta];
		vparam[4] = params[SOIdentification::wn];
		nh.setParam(dofName[i]+"_identified_params", vparam);
		//Stop identification
		axis_control[i] = disableAxis;
		ident[i].reset();
		//Report
		ROS_INFO("Stoped indentification of %s DOF.",dofName[i].c_str());
		ROS_INFO("Identified parameters: %f %f %f %f",
				params[SOIdentification::alpha],
				params[SOIdentification::kx],
				params[SOIdentification::kxx],
				params[SOIdentification::delta],
				params[SOIdentification::wn]);
		//Update dynamic parameters
		this->updateDynRecConfig();

		return 0;
	}

	if (i>=3)
	{
		return ident[i]->step(labust::math::wrapRad(labust::math::wrapRad(ident[i]->Ref())-labust::math::wrapRad(measurement[i])),Ts);
	}

	return ident[i]->step(ident[i]->Ref()-measurement[i],Ts);
}

void VelocityControl::step()
{
	auv_msgs::BodyForceReq tau, tauach;
	this->safetyTest();

	for (int i=u; i<=r;++i)
	{
		//If some of the axis are timed-out just ignore
		if (suspend_axis[i])
		{
			controller[i].output = 0;
			continue;
		}

		switch (axis_control[i])
		{
		case manualAxis:
			controller[i].output = tauManual[i];
			break;
		case controlAxis:
			PIFFController_step(&controller[i], Ts);
			break;
		case identAxis:
			controller[i].output = doIdentification(i);
			break;
		case disableAxis:
		default:
			controller[i].output = 0;
			break;
		}
	}
	//Copy to tau
	//tau.wrench.force.x = controller[u].output;
	if (axis_control[u] == controlAxis)
	{
		double ulimit(1.0);
		if (controller[u].autoTracking) ulimit = controller[u].outputLimit;
		if (fabs(controller[u].desired) > ulimit) controller[u].desired = controller[u].desired/fabs(controller[u].desired)*ulimit;
		tau.wrench.force.x = controller[u].desired;
	}
	else
	{
		tau.wrench.force.x = controller[u].output;
	}
	tau.wrench.force.y = controller[v].output;
	tau.wrench.force.z = controller[w].output;
	tau.wrench.torque.x = controller[p].output;
	tau.wrench.torque.y = controller[q].output;
	tau.wrench.torque.z = controller[r].output;

	tauach=tau;

	if (controller[u].autoTracking) tauach.disable_axis.x = controller[u].windup;
	if (controller[v].autoTracking) tauach.disable_axis.y = controller[v].windup;
	if (controller[w].autoTracking) tauach.disable_axis.z = controller[w].windup;
	if (controller[p].autoTracking) tauach.disable_axis.roll = controller[p].windup;
	if (controller[q].autoTracking) tauach.disable_axis.pitch = controller[q].windup;
	if (controller[r].autoTracking) tauach.disable_axis.yaw = controller[r].windup;

	//Restart values
	//newReference = newEstimate = false;
	tauach.header.stamp = tau.header.stamp = lastEst;
	tauAchW.publish(tauach);
	tauOut.publish(tau);
}

void VelocityControl::start()
{
	ros::Rate rate(1/Ts);

	while (nh.ok())
	{
		ros::spinOnce();
		step();
		rate.sleep();
	}
}

void VelocityControl::initialize_controller()
{
	ROS_INFO("Initializing velocity controller...");

	Eigen::Vector6d closedLoopFreq(Eigen::Vector6d::Ones());
	labust::tools::getMatrixParam(nh,"velocity_controller/closed_loop_freq", closedLoopFreq);
	Eigen::Vector6d outputLimit(Eigen::Vector6d::Zero());
	labust::tools::getMatrixParam(nh,"velocity_controller/output_limits", outputLimit);
	Eigen::Vector6d disAxis(Eigen::Vector6d::Ones());
	labust::tools::getMatrixParam(nh,"velocity_controller/disable_axis", disAxis);
	Eigen::Vector6d manAxis(Eigen::Vector6d::Ones());
	labust::tools::getMatrixParam(nh,"velocity_controller/manual_axis", manAxis);
	Eigen::Vector6d autoTracking(Eigen::Vector6d::Zero());
	labust::tools::getMatrixParam(nh,"velocity_controller/auto_tracking", autoTracking);

	labust::tools::DynamicsModel model(nh);
	Eigen::Vector6d alphas(model.added_mass);
	Eigen::Vector6d alpha_mass;
	alpha_mass<<model.mass,model.mass,model.mass,
			model.inertia_matrix.diagonal();
	alphas += alpha_mass;

	nh.param("velocity_controller/period",Ts,Ts);

	for (int32_t i = u; i <=r; ++i)
	{
		PIDController_init(&controller[i]);
		controller[i].closedLoopFreq = closedLoopFreq(i);
		controller[i].outputLimit = outputLimit(i);
		controller[i].modelParams[alpha] = alphas(i);
		controller[i].modelParams[beta] = model.damping(i);
		controller[i].modelParams[betaa] = model.qdamping(i);
		PIFFController_tune(&controller[i]);
		controller[i].autoTracking = autoTracking(i);

		if (!manAxis(i)) axis_control[i] = controlAxis;
		if (manAxis(i)) axis_control[i] = manualAxis;
		if (disAxis(i)) axis_control[i] = disableAxis;
		suspend_axis[i]=false;

		ph.setParam(dofName[i]+"_Kp",controller[i].gains[Kp]);
		ph.setParam(dofName[i]+"_Ki",controller[i].gains[Ki]);

		ROS_INFO("Controller %d:",i);
		ROS_INFO("ModelParams: %f %f %f",controller[i].modelParams[alpha], controller[i].modelParams[beta],
				controller[i].modelParams[betaa]);
		ROS_INFO("Gains: %f %f %f",controller[i].gains[Kp], controller[i].gains[Ki],
				controller[i].gains[Kt]);
	}

	ROS_INFO("Velocity controller initialized.");
}
