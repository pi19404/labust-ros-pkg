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
#include <labust/missionControl/missionControl.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>
#include <boost/algorithm/string.hpp>

int main(int argc, char* argv[])
try
{
	labust::xml::ReaderPtr reader(new labust::xml::Reader("../conf/missionControl.xml",true));
	reader->useNode(reader->value<_xmlNode*>("/configurations/config[@name='pMissionControl']"));

	std::string paramName, paramValue;

	labust::control::MissionControl control(reader);
	std::cout<<"Beginning test of Mission Control Petri Net"<<std::endl;
	std::cout<<"Ensure program finishes in \"Success\" state!"<<std::endl;

	std::map<std::string,std::string> missionCommand, data;

	std::cout<<"Output:"<<std::endl;
	missionCommand = control.GetCurrentMissionCommand();
	for(std::map<std::string,std::string>::const_iterator command = missionCommand.begin(); command!=missionCommand.end(); command++)
	{
		std::cout<<command->first<<": "<<command->second<<std::endl;
	}

	data["Mode"]="Test1";
	control.AddOtherData(data);
	std::cout<<std::endl<<"Output:"<<std::endl;	
	missionCommand = control.GetCurrentMissionCommand();
	for(std::map<std::string,std::string>::const_iterator command = missionCommand.begin(); command!=missionCommand.end(); command++)
	{
		std::cout<<command->first<<": "<<command->second<<std::endl;
	}

	data["Mode"]="Test2";
	data["Var1"]="Testing";
	data["Var2"]="Testing_too";	
	control.AddOtherData(data);
	std::cout<<std::endl<<"Output:"<<std::endl;	
	missionCommand = control.GetCurrentMissionCommand();
	for(std::map<std::string,std::string>::const_iterator command = missionCommand.begin(); command!=missionCommand.end(); command++)
	{
		std::cout<<command->first<<": "<<command->second<<std::endl;
	}

	data["Mode"]="Success";
	data["Var1"]="19";
	data["Var2"]="31";
	data["Var3"]="11";

	control.AddOtherData(data);
	std::cout<<std::endl<<"Output:"<<std::endl;	
	missionCommand = control.GetCurrentMissionCommand();
	for(std::map<std::string,std::string>::const_iterator command = missionCommand.begin(); command!=missionCommand.end(); command++)
	{
		std::cout<<command->first<<": "<<command->second<<std::endl;
	}	

	if(!boost::iequals(missionCommand["State"],"Success"))
	{
		std::cout<<"WARNING - test did not complete, check messages above!"<<std::endl;
	}
	else
	{
		std::cout<<"Test successful - MissionControl works properly."<<std::endl;
	}	

	std::cout<<"Program terminated"<<std::endl;
	return 0;
}
catch (std::exception& e)
{
	std::cout<<e.what()<<std::endl;
	return -1;	
}
