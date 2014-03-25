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
#include <labust/control/ControllerManager.hpp>

#include <navcon_msgs/ControllerState.h>

using namespace labust::control;

ControllerManager::ControllerManager(){this->onInit();};

void ControllerManager::onInit()
{
	ros::NodeHandle nh;
	//Setup services
	activateController = nh.advertiseService("activate_controller",
			&ControllerManager::onActivateController, this);
	registerController = nh.advertiseService("register_controller",
			&ControllerManager::onRegisterController, this);

	//Setup publisher
	controllerState = nh.advertise<navcon_msgs::ControllerState>("controller_state",1);
}

bool ControllerManager::onActivateController(navcon_msgs::ControllerSelect::Request& req,
				navcon_msgs::ControllerSelect::Response& resp)
{
	for (int i=0; i<req.name.size();++i) req.state[i]?activate(req.name[i]):deactivate(req.name[i]);

	return true;
}

bool ControllerManager::activate(const std::string& name)
{
	ControllerMap::const_iterator it = controllers.find(name);
	if (it == controllers.end()) return false;

	//Check if already active
	boost::array<std::string, 6>::const_iterator tIt,nIt;
	tIt = std::find(actTau.begin(), actTau.end(), name);
	nIt = std::find(actNu.begin(), actNu.end(), name);
	if ((tIt != actTau.end()) || (nIt != actNu.end())) return true;

	ControllerMap::mapped_type& info = it->second;
	std::set<std::string> deactivationList;

	for (int i=0; i<dofsNum; ++i)
	{
		//If the DOF is used and already ocuppied
		bool usedT(info.used_tau[i] == 1);
		bool usedN(info.used_nu[i] == 1);
		bool occupied(!actTau[i].empty());

		if (usedT && occupied) deactivationList.insert(actTau[i]);
		//if (actTau[i] != "")
	}

}

bool ControllerManager::onRegisterController(navcon_msgs::RegisterController_v2::Request& req,
		navcon_msgs::RegisterController_v2::Response& resp)
{
	if (controllers.find(req.name) != controllers.end())
	{
		ROS_WARN("Controller with name %s already exists.", req.name.c_str());
	}
	controllers[req.name] = req;
	return true;
}
