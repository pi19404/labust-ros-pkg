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
 *  Created: 30.10.2013.
 *********************************************************************/
#include <labust/control/IdentificationNode.hpp>
#include <labust/math/NumberManipulation.hpp>
#include <labust/tools/conversions.hpp>

#include <auv_msgs/BodyForceReq.h>

#include <boost/bind.hpp>

using namespace labust::control;

IdentificationNode::IdentificationNode():
				measurements(Eigen::Matrix<double,6,1>::Zero()),
				integrateUV(true)
{
	this->onInit();
}

void IdentificationNode::onInit()
{
	ros::NodeHandle nh,ph("~");
	aserver.reset(new ActionServer(nh,
			"Identification",
			boost::bind(&IdentificationNode::doIdentification, this, _1),
			false));
	aserver->start();

	tauOut = nh.advertise<auv_msgs::BodyForceReq>("tauOut", 1);
	meas = nh.subscribe<auv_msgs::NavSts>("meas", 1,
			&IdentificationNode::onMeasurement,this);

	ph.param("integrateUV",integrateUV, true);
}

void IdentificationNode::onMeasurement(const auv_msgs::NavSts::ConstPtr& meas)
{
	static ros::Time lastSampleTime=ros::Time::now();

	if (integrateUV)
	{
		boost::mutex::scoped_lock l(measmux);
		double dT = (ros::Time::now() - lastSampleTime).toSec();
		lastSampleTime = ros::Time::now();
		ROS_INFO("Estimated rate: %f",dT);
		measurements(x) += meas->body_velocity.x*dT;
		measurements(y) += meas->body_velocity.y*dT;
	}
	else
	{
		boost::mutex::scoped_lock l(measmux);
		measurements(x) = meas->position.north;
		measurements(y) = meas->position.east;
	}
	measurements(z) = meas->position.depth;
	measurements(roll) = meas->orientation.roll;
	measurements(pitch) = meas->orientation.pitch;
	measurements(yaw) = meas->orientation.yaw;
}

void IdentificationNode::doIdentification(const Goal::ConstPtr& goal)
{
	ROS_INFO("Recevied goal.");

	//Some run-time sanity checking.
	if ((goal->sampling_rate <= 0) ||
			(goal->command == 0) ||
			(goal->hysteresis == 0) ||
			(goal->dof < 0) || (goal->dof > goal->Yaw))
	{
		ROS_ERROR("Some identification criteria are faulty: "
				"command: (%f != 0), "
				"hysteresis: (%f != 0), "
				"dof: (abs(%d) < 6), "
				"(sampling_rate: (%f >=0)", goal->command,
				goal->hysteresis, goal->dof, goal->sampling_rate);
		return;
	}

	ros::Rate rate(goal->sampling_rate);
	int oscnum(0);

	SOIdentification ident;
	ident.setRelay(goal->command,goal->hysteresis);
	ident.Ref(goal->reference);

	ROS_INFO("Started identification of %d DOF.",goal->dof);

	//Reset the integrals
	if (integrateUV)
	{
		boost::mutex::scoped_lock l(measmux);
		measurements(x)=0;
		measurements(y)=0;
	}

	while (!ident.isFinished())
	{
		double error = goal->reference - measurements(goal->dof);
		//Handle angular values
		if (goal->dof >= roll)
		{
			error = labust::math::wrapRad(
					labust::math::wrapRad(goal->reference) -
					labust::math::wrapRad(measurements(goal->dof)));
		}
		//Perform one step
		this->setTau(goal->dof, ident.step(error, 1/goal->sampling_rate));

		//Check for preemption
		if (aserver->isPreemptRequested() || !ros::ok())
		{
			ROS_INFO("DOFIdentification for %d: Preempted", goal->dof);
			//Set output to zero
			this->setTau(goal->dof, 0.0);
			// set the action state to preempted
			aserver->setPreempted();
			return;
		}

		//Send feedback about progress
		if (ident.hasSwitched())
		{
			navcon_msgs::DOFIdentificationFeedback feedback;
			feedback.dof = goal->dof;
			feedback.error = ident.avgError();
			feedback.oscillation_num = ++oscnum/2;
		}

		//Set the feedback value
		rate.sleep();
	}

	//Stop the vessel
	this->setTau(goal->dof, 0.0);

	if (ident.isFinished())
	{
		const std::vector<double>& params = ident.parameters();
		Result result;
		result.alpha = params[SOIdentification::alpha];
		result.beta = params[SOIdentification::kx];
		result.betaa = params[SOIdentification::kxx];
		result.delta = params[SOIdentification::delta];
		result.wn = params[SOIdentification::wn];

		ROS_INFO("Identified parameters: %f %f %f %f",
				params[SOIdentification::alpha],
				params[SOIdentification::kx],
				params[SOIdentification::kxx],
				params[SOIdentification::delta],
				params[SOIdentification::wn]);

		// set the action state to succeeded
		aserver->setSucceeded(result);
	}
}

void IdentificationNode::setTau(int elem, double value)
{
	labust::simulation::vector tauvec = labust::simulation::vector::Zero();
	tauvec(elem) = value;
	std::ostringstream out("ident_");
	out<<elem;
	auv_msgs::BodyForceReqPtr tau(new auv_msgs::BodyForceReq());
	tau->header.stamp = ros::Time::now();
	tau->goal.requester = out.str();
	labust::tools::vectorToPoint(tauvec,tau->wrench.force);
	labust::tools::vectorToPoint(tauvec,tau->wrench.torque,3);
	tauOut.publish(tau);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"ident_node");
	labust::control::IdentificationNode ident;
	ros::spin();
	return 0;
}
