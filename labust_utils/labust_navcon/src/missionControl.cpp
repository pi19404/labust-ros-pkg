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
#include <labust/xml/xmlfwd.hpp>
#include <labust/xml/XMLReader.hpp>
#include <boost/algorithm/string.hpp>
#include <labust/tools/TimingTools.hpp>

using namespace labust;
using namespace control;

MissionStateTransition::MissionStateTransition(const labust::xml::ReaderPtr reader)
{
	labust::xml::xmlNodePtr currentNode= NULL;
	currentNode = reader->currentNode();
	
	std::string newState;
	std::vector<std::string> preconditions;			
	if(reader->try_value("@state",&newState))
	{
		state = newState;
		labust::xml::NodeCollectionPtr preconditionNodes;
		std::vector<std::string> preconditions;
		try
		{
			preconditionNodes = reader->value<labust::xml::NodeCollectionPtr>("param");	
			for(unsigned int preconditionId = 0; preconditionId < preconditionNodes->size(); preconditionId++)
			{
				reader->useNode((*preconditionNodes)[preconditionId]);
				std::string paramName, paramValue, preconditionString;								
					
				if(reader->try_value("@name",&paramName) && reader->try_value("@value",&paramValue))
				{
					preconditionStruct transitionPrecondition;
					transitionPrecondition.tolerance = 0;
					transitionPrecondition.paramName = paramName;
					switch(paramValue[0])
					{
						case '<':
						if(paramValue[1]=='=')
						{
							transitionPrecondition.type=lessEqual;	
						}
						else
						{
							transitionPrecondition.type=less;
						}
						break;
						case '>':
						if(paramValue[1]=='=')
						{
							transitionPrecondition.type=moreEqual;	
						}
						else
						{
							transitionPrecondition.type=more;
						}
						break;
						case '!':
							transitionPrecondition.type=nEqual;
						break;
						default:
							reader->try_value("@tolerance",&transitionPrecondition.tolerance);
							transitionPrecondition.type=equal;;
						break;
					}
					switch(transitionPrecondition.type)
					{
						case less: //fallthrough intended
						case more: //fallthrough intended
						case nEqual:
							transitionPrecondition.paramValue=paramValue.substr(1); 
						break;
						case lessEqual: //fallthrough intended
						case moreEqual: //fallthrough intended
							transitionPrecondition.paramValue=paramValue.substr(2);
						break;
						default:
							transitionPrecondition.paramValue=paramValue;
						break;			
					}
					transitionPrecondition.fulfilled = false;
					transitionPreconditions.push_back(transitionPrecondition);
				}				
			}			
		}
		catch (labust::xml::XMLException &e)
		{
			//no problem if they don't exist - states can be without transition preconditions
			std::cerr<<"Warning, transition to state \""<<newState<<"\" has no preconditions. ("<<e.what()<<")"<<std::endl;
		};
	}		
	else
	{
		std::stringstream buffer;
		buffer<<"Warning, transition to state \""<<newState<<"\" has undefined final state and was discarded";
		throw labust::xml::XMLException(buffer.str());
	}
	reader->useNode(currentNode);
}

bool MissionStateTransition::ProcessSymbol(const std::string &paramName, const std::string &paramValue)
{
	bool triggerable = true;
	bool symbolUsed = false; // this to prevent one symbol to trigger multiple preconditions (and enable the preconditions to require the same symbol to be received multiple times)
	double numericSymbolValue;
	{
		std::stringstream buffer;
		buffer<<paramValue;
		buffer>>numericSymbolValue;
	}	

	for(std::vector<preconditionStruct>::iterator precondition = transitionPreconditions.begin(); precondition!=transitionPreconditions.end(); precondition ++)
	{
		if(!precondition->fulfilled && !symbolUsed && boost::iequals(paramName,precondition->paramName))
		{
			double numericPreconditionValue;
			if(precondition->tolerance!=0 || (precondition->type!=equal && precondition->type!=nEqual))
			{
				std::stringstream buffer;
				buffer<<precondition->paramValue;
				buffer>>numericPreconditionValue;
			}
			switch(precondition->type)
			{
				case less:
					precondition->fulfilled = numericSymbolValue < numericPreconditionValue;
				break;
				case more:
					precondition->fulfilled = numericSymbolValue > numericPreconditionValue;
				break;
				case lessEqual:
					precondition->fulfilled = numericSymbolValue <= numericPreconditionValue;
				break;
				case moreEqual: 
					precondition->fulfilled = numericSymbolValue >= numericPreconditionValue;
				break;
				case nEqual:
					if(precondition->tolerance!=0)
					{
						precondition->fulfilled = abs(numericSymbolValue-numericPreconditionValue)>precondition->tolerance; 
					}
					else
					{
						precondition->fulfilled = !boost::iequals(precondition->paramValue,paramValue); 
					}				
				break;
				case equal: //fallthrough intended				
				default:
					if(precondition->tolerance!=0)
					{
						precondition->fulfilled = abs(numericSymbolValue-numericPreconditionValue)<precondition->tolerance; 
					}
					else
					{
						precondition->fulfilled = boost::iequals(precondition->paramValue,paramValue);
					}
				break;			
			}	
			symbolUsed = precondition->fulfilled;	
		}
		triggerable = triggerable && precondition->fulfilled;
	}
	if(triggerable)
	{
		Reset();
	}
	return triggerable;
}

std::string MissionStateTransition::GetState()
{
	return state;
}

void MissionStateTransition::Reset()
{
	for(std::vector<preconditionStruct>::iterator precondition = transitionPreconditions.begin(); precondition  != transitionPreconditions.end(); precondition ++)
	{
		precondition->fulfilled = false;
	}
}

MissionState::MissionState(const labust::xml::ReaderPtr reader, const std::string &stateName)
{
		std::string configQuery = (stateName.empty() ? "state" : "state[@name='"+stateName+"']");

		labust::xml::xmlNodePtr configNode= NULL, currentNode = NULL;
		currentNode = reader->currentNode();

		try 
   	{
			configNode = reader->value<labust::xml::xmlNodePtr>(configQuery);
    		reader->useNode(configNode);		
			reader->try_value("@name", &name);
			try
			{
				labust::xml::NodeCollectionPtr outputParamNodes;
				outputParamNodes = reader->value<labust::xml::NodeCollectionPtr>("output/param");
				for(unsigned int paramId = 0; paramId<outputParamNodes->size(); paramId++)
				{
					reader->useNode((*outputParamNodes)[paramId]);
					std::string paramName, paramValue;							
					if(reader->try_value("@name",&paramName) && reader->try_value("@value",&paramValue))
					{	
						output[paramName]=paramValue;
					}				
				} 				
			}
			catch (labust::xml::XMLException &e)
			{
				//no problem if they don't exist - states can be without transitions or states
				std::cerr<<"Warning, state \""<<stateName<<"\" has no output. ("<<e.what()<<")"<<std::endl;
			}			

			reader->useNode(configNode);	

			try
			{
				labust::xml::NodeCollectionPtr transitionNodes;
				transitionNodes = reader->value<labust::xml::NodeCollectionPtr>("transition");
				for(unsigned int transitionId = 0; transitionId<transitionNodes->size(); transitionId++)
				{
					reader->useNode((*transitionNodes)[transitionId]);
					try
					{
						MissionStateTransition transition(reader);
						transitions.push_back(transition); 
					}
					catch (labust::xml::XMLException &e)
					{
						std::cerr<<e.what()<<std::endl;
					}				
				}	
			}
			catch (labust::xml::XMLException &e)
			{
				//no problem if they don't exist - states can be without transitions or states
				std::cerr<<"Warning, state \""<<stateName<<"\" has no transitions. ("<<e.what()<<")"<<std::endl;
			}

			std::cout<<"State \""<<name<<"\" loaded successfully, "<<transitions.size()<<" transitions and output with "<<output.size()<<" params."<<std::endl;
		}
		catch (labust::xml::XMLException &e)
		{
			std::cerr<<"Warning, problem loading state \""<<stateName<<"\". ("<<e.what()<<")"<<std::endl;
		}
		reader->useNode(currentNode);
}

std::string MissionState::ProcessSymbol(const std::string &paramName, const std::string &paramValue)
{
	std::string newStateName = "";
	for(std::vector<MissionStateTransition>::iterator transition = transitions.begin(); transition != transitions.end(); transition++)
	{
		if(transition->ProcessSymbol(paramName, paramValue))
		{
			newStateName = transition->GetState();		
			break;
		}		
	}
	return newStateName;
}

std::map<std::string,std::string> MissionState::GetOutput()
{
	return output;
}

MissionControl::MissionControl(const labust::xml::ReaderPtr reader, const std::string& configToUse):
running(true)
{
	std::cout<<std::endl<<"Loading config for Mission Control"<<std::endl;
	std::string configQuery = (configToUse.empty() ? "moduleConfig[@type='LabustMissionControl']" : "moduleConfig[@name='"+configToUse+"' AND @type='LabustMissionControl']");	

	labust::xml::xmlNodePtr currentNode = NULL;
	currentNode = reader->currentNode();
	
	try
   {
		reader->useNode(reader->value<labust::xml::xmlNodePtr>(configQuery));
		LoadConfig(reader);
	}
	catch(labust::xml::XMLException &e)
	{
		std::cerr<<"Error loading config for Mission Control. ("<<e.what()<<")"<<std::endl;
	}
	reader->useNode(currentNode);
}

MissionControl::MissionControl(const std::string& filePath, const std::string& configToUse):
running(true)
{
	std::cout<<std::endl<<"Loading config for  Mission Control"<<std::endl;
	labust::xml::ReaderPtr reader(new labust::xml::Reader(filePath,true));
	std::string configQuery = (configToUse.empty() ? "//moduleConfig[@type='LabustMissionControl']" : "//moduleConfig[@name='"+configToUse+"' AND @type='LabustMissionControl']");	

	try
   {
		reader->useNode(reader->value<labust::xml::xmlNodePtr>(configQuery));
		LoadConfig(reader);		
	}
	catch(labust::xml::XMLException &e)
	{
		std::cerr<<"Error loading config for Mission Control. ("<<e.what()<<")"<<std::endl;
	}	
}

MissionControl::~MissionControl()
{
	std::cout<<"Stopping mission control timer..."<<std::endl;
	running = false;
	timerThread.join();
	std::cout<<"Mission control timer stopped."<<std::endl;
}

void MissionControl::LoadConfig(const labust::xml::ReaderPtr reader)
{
	labust::xml::xmlNodePtr statesNode = reader->currentNode();		
	std::string currentStateName = "";
	try
	{
		labust::xml::NodeCollectionPtr stateNodes;
		stateNodes = reader->value<labust::xml::NodeCollectionPtr>("state");	
		for(unsigned int stateId = 0; stateId<stateNodes->size(); stateId++)
		{
			reader->useNode((*stateNodes)[stateId]);
			std::string stateName;							
			if(reader->try_value("@name",&stateName))
			{	
				if(currentStateName.empty())
				{
					std::string initial;
					reader->try_value("@initial",&initial);
					if(boost::iequals(initial,"true"))
					{
						currentStateName = stateName;
					}
				}
				reader->useNode(statesNode);
				MissionState newState(reader,stateName);
				allStates.insert(std::pair<std::string, MissionState>(stateName,newState));
			}				
		} 		
		if(currentStateName.empty())
		{
			reader->useNode((*stateNodes)[0]);
			reader->value("@name",&currentStateName);
			std::cerr<<"Warning - no inital state specified, assuming first one ("<<currentStateName<<")"<<std::endl;
		}	
	}
	catch (labust::xml::XMLException &e)
	{
		//Warn if no states in module
		std::cerr<<"Warning, no state nodes found. ("<<e.what()<<")"<<std::endl;
	}
	

	currentState = allStates.find(currentStateName);
	timerSeconds = 0;	
	boost::thread(boost::bind(&MissionControl::TimerThreadFunction,this));
	if(currentState!=allStates.end())
	{
		std::cout<<" Mission Control configuration loading completed, initial state: \""<<currentState->first<<"\"."<<std::endl<<std::endl;
	}
	else
	{
		std::cerr<<"Unable to set initial state"<<std::endl;//change this to exception!
	}
}

bool MissionControl::AddVehicleState(const labust::vehicles::stateMap& vehicleState)
{
	bool retVal = false;
	for(labust::vehicles::stateMap::const_iterator stateElement = vehicleState.begin(); stateElement!= vehicleState.end(); stateElement++)
	{
		std::string paramName;
		switch(stateElement->first)
		{
			using namespace vehicles;
			case state::u:
				paramName="NU.u";
			break;
			case state::v:
				paramName="NU.v";
			break;
			case state::w:
				paramName="NU.w";
			break;
			case state::p:
				paramName="NU.p";
			break;
			case state::q:
				paramName="NU.q";
			break;
			case state::r:
				paramName="NU.r";
			break;

			case state::x:
				paramName="ETA.x";
			break;
			case state::y:
				paramName="ETA.y";
			break;
			case state::z:
				paramName="ETA.z";
			break;
			case state::roll:
				paramName="ETA.roll";
			break;
			case state::pitch:
				paramName="ETA.pitch";
			break;
			case state::yaw:
				paramName="ETA.yaw";
			break;

			case state::lat:
				paramName="GEO.lat";
			break;
			case state::lon:
				paramName="GEO.lon";
			break;
			case state::heading:
				paramName="GEO.heading";
			break;

			case state::dH:
				paramName="LF.dH";
			break;
			case state::dV:
				paramName="LF.dV";
			break;
		
			case state::depthPressure:
				paramName="SEN.depthPressure";
			break;
			default:
				paramName="UNKNOWN_VEHICLE_STATE";
		}

		std::stringstream paramValue;
		paramValue<<stateElement->second;
	
		std::string newStateName = currentState->second.ProcessSymbol(paramName,paramValue.str());
		if(!newStateName.empty())
		{
			std::map<std::string,MissionState>::iterator newState = allStates.find(newStateName);
			if (newState!=allStates.end())
			{			
				std::cout<<"Changed state from state "<<currentState->first<<" to "<<newStateName<<"."<<std::endl;
				boost::mutex::scoped_lock stateLock(stateLocker);				
				currentState=newState;
				stateLock.unlock();
				boost::mutex::scoped_lock timerLock(timerLocker);
				timerSeconds = 0;	
				timerLock.unlock();
				retVal = true;
			}
			else
			{
				std::cerr<<"Error changing state from state "<<currentState->first<<" to "<<newStateName<<", new state does not exist!"<<std::endl;
			}
		}
	}
	return retVal;
}

bool MissionControl::AddOtherData(const std::map<std::string,std::string>& data)
{
	bool retVal = false;
	for(std::map<std::string,std::string>::const_iterator dataElement = data.begin(); dataElement!= data.end(); dataElement++)
	{	
		std::string newStateName = currentState->second.ProcessSymbol(dataElement->first, dataElement->second);
		if(!newStateName.empty())
		{
			std::map<std::string,MissionState>::iterator newState = allStates.find(newStateName);
			if (newState!=allStates.end())
			{			
				std::cout<<"Changed state from state "<<currentState->first<<" to "<<newStateName<<"."<<std::endl;
				boost::mutex::scoped_lock stateLock(stateLocker);			
				currentState=newState;				
				stateLock.unlock();
				boost::mutex::scoped_lock timerLock(timerLocker);
				timerSeconds = 0;	
				timerLock.unlock();
				retVal = true;
			}
			else
			{
				std::cerr<<"Error changing state from state "<<currentState->first<<" to "<<newStateName<<", new state does not exist!"<<std::endl;
			}
		}
	}
	return retVal;
}

std::map<std::string,std::string> MissionControl::GetCurrentMissionCommand()
{
	boost::mutex::scoped_lock lock(stateLocker);
	return currentState->second.GetOutput();
}

void MissionControl::TimerThreadFunction()
{
	while(running)
	{
		static std::map<std::string,std::string> timerData;
		static double lastTimerTick;
		lastTimerTick = labust::tools::unix_time();
		static std::stringstream converter;	
		converter.str("");
		converter.clear();
		boost::mutex::scoped_lock timerLock(timerLocker);
		converter<<timerSeconds;
		timerSeconds++;
		timerLock.unlock();
		timerData["#timer"]=converter.str();	
		AddOtherData(timerData);
		double elapsedTime = labust::tools::unix_time() - lastTimerTick;
		if(elapsedTime<1)
		{
			int waitms = 1000 - (int)(1000 * elapsedTime);
			boost::this_thread::sleep(boost::posix_time::milliseconds(waitms)); 
		}
	}	
}


