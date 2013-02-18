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
*  Created: 12.02.2013.
*********************************************************************/
#include <labust/vehicles/VehicleApp.hpp>
#include <labust/vehicles/VehicleDriver.hpp>
#include <labust/vehicles/VehicleFactoryName.hpp>
#include <labust/xml/GyrosReader.hpp>
#include <labust/xml/GyrosWriter.hpp>

using namespace labust::vehicles;

VehicleApp::VehicleApp():
		nhandle(),
		phandle("~")
{
	//Find plugin configuration
	std::string pluginName, pluginConfig, pluginId;
	phandle.param("PluginName",pluginName, pluginName);
	phandle.param("PluginConfig",pluginConfig,pluginConfig);
	if (pluginName.empty() || pluginConfig.empty())
	{
		throw std::runtime_error("Plugin name and configuration missing.");
	}
	phandle.param("PluginId",pluginId,pluginId);
	this->loadPlugin(pluginName, pluginConfig, pluginId);

	//Handle publisher names.
	std::string stateName("state"), dataName("data");
	phandle.param("StateOut",stateName,stateName);
	phandle.param("DataOut",dataName,dataName);
	this->publish(stateName, dataName);
	//Handle subscriber names.
	std::string tauName("tau"), cmdName("cmd");
	phandle.param("TauIn",tauName,tauName);
	phandle.param("CommandIn",cmdName,cmdName);
	this->subscribe(tauName, cmdName);

	std::cerr<<"VehicleApp successfully configured."<<std::endl;
}

void VehicleApp::loadPlugin(const std::string& pluginName, const std::string& pluginConfig, const std::string& pluginId)
{
	//Release the vehicle driver.
	uuv.reset();
	//Release and load the plugin.
	plugin.reset(new labust::vehicles::VehiclePlugin(pluginName,FactoryCreatorName::value));
	//Load the XML configuration.
	labust::xml::ReaderPtr reader(new labust::xml::Reader(pluginConfig,true));
	reader->useNode(reader->value<_xmlNode*>("//configurations"));
	//Instantiate the driver.
	uuv.reset((*plugin)(reader));
}

void VehicleApp::subscribe(const std::string& tauName, const std::string& cmdName)
{
	tau = nhandle.subscribe<std_msgs::String>(tauName,msgBuf,boost::bind(&VehicleApp::onTau,this,_1));
	cmd = nhandle.subscribe<std_msgs::String>(tauName,msgBuf,boost::bind(&VehicleApp::onCmd,this,_1));
}

void VehicleApp::publish(const std::string& stateName, const std::string& dataName)
{
	state = nhandle.advertise<std_msgs::String>(stateName,msgBuf);
	data = nhandle.advertise<std_msgs::String>(dataName,msgBuf);
}

void VehicleApp::onTau(const std_msgs::String::ConstPtr& tau)
{
	std::cerr<<"Tau received."<<std::endl;
	labust::xml::GyrosReader reader(tau->data);
	std::map<int,double> tauM;
	reader.dictionary(tauM);

	ROS_INFO("Tau decoded:%f",tauM[labust::vehicles::tau::N]);

	labust::vehicles::tauMap tauC;
	for (size_t i=labust::vehicles::tau::X;
			i<=labust::vehicles::tau::N; ++i) tauC[i] = tauM[i];

	ROS_INFO("Tau setting:%f",tauC[labust::vehicles::tau::N]);

	uuv->setTAU(tauC);

	std::cerr<<"Tau set."<<std::endl;

	labust::vehicles::stateMapPtr state(new labust::vehicles::stateMap());
	uuv->getState(*state);
	std::cerr<<"State:"<<(*state)[labust::vehicles::state::yaw]<<std::endl;
	//(*state)[labust::vehicles::state::x] = time;
	labust::xml::GyrosWriter writer(state->begin(),state->end());
	writer.SetTimeStamp(true);
	std_msgs::StringPtr str(new std_msgs::String());
	str->data = writer.GyrosXML();
	std::cerr<<"State publishing."<<std::endl;
	this->state.publish(str);

	std::cerr<<"State published."<<std::endl;
}

void VehicleApp::onCmd(const std_msgs::String::ConstPtr& cmd)
{
	uuv->setCommand(cmd->data);
}



