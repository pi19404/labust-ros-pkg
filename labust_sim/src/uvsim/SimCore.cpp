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
#include <labust/ros/SimCore.hpp>
#include <labust/tools/DynamicsLoader.hpp>

#include <sstream>

using namespace labust::simulation;

void labust::simulation::configureModel(const ros::NodeHandle& nh, RBModel& model)
{
	labust::tools::loadDynamicsParams(nh, model);

	nh.param("sampling_time",model.dT, model.dT);
	nh.param("coupled",model.isCoupled,model.isCoupled);
	Eigen::Vector3d bdg;
	labust::tools::getMatrixParam(nh,"bounding_ellipsoid",bdg);
	model.ae = bdg(0);
	model.be = bdg(1);
	model.ce = bdg(2);
	labust::tools::getMatrixParam(nh,"eta0",model.eta0);
	labust::tools::getMatrixParam(nh,"nu0",model.nu0);
	labust::tools::getMatrixParam(nh,"current",model.current);

	typedef Eigen::Matrix<double,6,1> NoiseVec;
	NoiseVec pn(NoiseVec::Zero()),mn(NoiseVec::Zero());
	labust::tools::getMatrixParam(nh,"process_noise",pn);
	labust::tools::getMatrixParam(nh,"measurement_noise",mn);
	model.noise.setNoiseParams(pn,mn);

	model.init();
}

SimCore::SimCore()
{
	this->onInit();
}

void SimCore::onInit()
{
	ros::NodeHandle nh;
	configureModel(nh, model);
	modelReport();
}

void SimCore::modelReport()
{
	ROS_INFO("Loaded the model:");
	ROS_INFO(" sampling-time: %f, mass: %f, gravity: %f, density: %f",
			model.dT, model.m, model.g_acc, model.rho);
	std::ostringstream str;
	str<<"["<<model.eta0.transpose()<<"], ["<<model.nu0.transpose()<<"]"<<std::endl;
	ROS_INFO(" (Eta0,Nu0): %s",str.str().c_str());
	str.str("");
	str<<"["<<model.current.transpose()<<"]";
	ROS_INFO(" Current: %s",str.str().c_str());
	ROS_INFO(" Bounding ellipsoid: (%f,%f,%f)",model.ae, model.be, model.ce);
	ROS_INFO(" Is coupled model: %d",model.isCoupled);
	str.str("");
	str<<model.Io<<"\n"<<model.Ma<<"\n"<<model.Dlin<<"\n"<<model.Dquad;
	ROS_INFO("(Io,Ma,Dlin,Dquad):\n%s",str.str().c_str());
}
