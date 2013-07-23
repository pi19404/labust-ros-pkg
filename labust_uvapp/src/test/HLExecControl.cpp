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
 *  Created: 11.07.2013.
 *********************************************************************/
#include "HLExecControl.hpp"
#include <labust_uvapp/EnableControl.h>

using namespace labust::control;
HLExecControl::HLExecControl(){this->onInit();};

void HLExecControl::onInit()
{
	for (int i=0; i<active_dofs.size(); ++i)	 active_dofs[i]="";
	ros::NodeHandle nh;
	//Configure service
	registerServer = nh.advertiseService("RegisterController",
			&HLExecControl::onRegReq, this);

	nuIn = nh.subscribe<auv_msgs::BodyVelocityReq>("nuRef", 1,
			&HLExecControl::onNuIn,this);
	cntSel = nh.subscribe<std_msgs::String>("controller_select", 1,
			&HLExecControl::onControllerSelect,this);

	nuRef = nh.advertise<auv_msgs::BodyVelocityReq>("nuRefMerged", 1);
}

bool HLExecControl::onRegReq(labust_uvapp::RegisterController::Request& req,
					labust_uvapp::RegisterController::Response& resp)
{
	std::string name(req.__connection_header->at("callerid"));
	ROS_INFO("Registration request from node %s.",req.info.name.c_str());
	for(int i=0; i<req.info.dofs.size(); ++i)
	{
		ROS_INFO("DOF %d.",req.info.dofs[i]);
	}
	controllers[req.info.name] = req.info;

	return true;
}

void HLExecControl::onControllerSelect(const std_msgs::String::ConstPtr& name)
{
	if (controllers.find(name->data) == controllers.end())
	{
		ROS_INFO("Controller %s was not registered.",name->data.c_str());
		return;
	}

	std::vector<std::string> disableControllers;
	std::vector<std::string> enableControllers;

	labust_uvapp::ControllerInfo& info = controllers[name->data];

	for(int i=0; i<info.dofs.size(); ++i)
	{
		int reqDOF = info.dofs[i];
		ROS_INFO("Degree of freedom %d",reqDOF);
		if (active_dofs[reqDOF] != "")
		{
			disableControllers.push_back(active_dofs[reqDOF]);
			//schedule all sub-controllers of the disable controller for disable
			disableControllers.insert(disableControllers.end(),
								controllers[active_dofs[reqDOF]].sub_controllers.begin(),
								controllers[active_dofs[reqDOF]].sub_controllers.end());
		 }
		active_dofs[reqDOF] = name->data;
	}

	enableControllers.push_back(name->data);
	for (int i=0; i<info.sub_controllers.size(); ++i)
	{
		//TEST IF REQUEIRED SUB-CONTROLLER IS REGISTERED
		enableControllers.push_back(info.sub_controllers[i]);
	}

	ros::NodeHandle nh;
	for (int i=0; i<disableControllers.size(); ++i)
	{
		ros::ServiceClient client = nh.serviceClient<labust_uvapp::EnableControl>(disableControllers[i] + "_enable");
		labust_uvapp::EnableControl srv;
		srv.request.enable = false;
		if (!client.call(srv))
		{
			ROS_ERROR("Failed to call the %s configuration service.",disableControllers[i].c_str());
		}
	};

	for (int i=0; i<enableControllers.size(); ++i)
	{
		ros::ServiceClient client = nh.serviceClient<labust_uvapp::EnableControl>(enableControllers[i] + "_enable");
		labust_uvapp::EnableControl srv;
		srv.request.enable = true;
		if (!client.call(srv))
		{
			ROS_ERROR("Failed to call the %s configuration service.",enableControllers[i].c_str());
		}
	};
}

void HLExecControl::onNuIn(const auv_msgs::BodyVelocityReq::ConstPtr& nu)
{
	std::string name(nu->__connection_header->at("callerid"));
	if (controllers.find(name) == controllers.end())
	{
		ROS_INFO("Controller %s was not registered.",name.c_str());
	}
	//auv_msgs::BodyVelocityReq::Ptr merged(new auv_msgs::BodyVelocityReq());
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"execution_control");
	//Initialize
	HLExecControl exec;
	//Start execution.
	ros::spin();
	return 0;
}



