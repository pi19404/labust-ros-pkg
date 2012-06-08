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
#include <labust/moos/DynRecMoosApp.hpp>

#include <boost/bind.hpp>

#include <iostream>

using namespace labust::moos;

DynRecMoosApp::DynRecMoosApp():
	connected(false),
	startUP(false){};

DynRecMoosApp::~DynRecMoosApp(){};

bool DynRecMoosApp::readMissionParameters()
{
	return true;
}

bool DynRecMoosApp::OnStartUp()
{
	std::cout << this->m_MissionReader.GetAppName() << ", starting ..." <<std::endl;

	if ((this->startUP = readMissionParameters()))
	{
		return registerMoosVariables();
	}

	return false;
}

bool DynRecMoosApp::OnConnectToServer()
{
	std::cout<< this->m_MissionReader.GetAppName()<< ", connected to server ..." <<std::endl;
	this->connected=true;

	return registerMoosVariables();
}

bool DynRecMoosApp::registerMoosVariables()
{
	if (this->connected && this->startUP)
	{
		m_Comms.Register("uuvapp_data",0);
	}

	return true;
}

bool DynRecMoosApp::OnNewMail(MOOSMSG_LIST& NewMail)
{
	this->UpdateMOOSVariables(NewMail);

	CMOOSMsg Msg;
	MOOSVARMAP::iterator end = this->m_MOOSVars.end();

	for (MOOSVARMAP::iterator it=m_MOOSVars.begin(); it != end; ++it)
	{
		if (it->second.IsFresh())
		{
			mediator->newMessage(it->second.GetName(),it->second.GetStringVal());
		}
	}

	return true;
}

bool DynRecMoosApp::Iterate()
{
	return true;
}

bool DynRecMoosApp::registerVariable(const std::string& name)
{
	boost::mutex::scoped_lock lock(registerVariableSync);
	this->AddMOOSVariable(name,name,name,0);
	return m_Comms.Register(name,0);
}

DynRecMoosApp::CommsCommands::CPtr DynRecMoosApp::getCommsCommands()
{
	CommsCommands::Ptr commands(new CommsCommands());
	//We can do that since the call is thread safe.
	commands->addRegisterVariableCallback(boost::bind(&DynRecMoosApp::registerVariable,this,_1));

	commands->addSendCommandCallback(boost::bind(
			static_cast< bool (CMOOSCommClient::*)
			(const std::string&, const std::string&, double)>(&CMOOSCommClient::Notify),
			&m_Comms,_1,_2,0));

	return commands;
}
