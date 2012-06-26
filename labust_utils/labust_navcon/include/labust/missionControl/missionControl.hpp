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
#ifndef MISSIONCONTROL_HPP_
#define MISSIONCONTROL_HPP_

#include <labust/xml/xmlfwd.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

namespace labust
{
	namespace control
	{
		enum preconditionType
		{
			equal=0, nEqual, lessEqual, less, more, moreEqual, range
		};
		typedef struct 
		{
			preconditionType type;
			std::string paramName, paramValue;
			bool fulfilled;
			double tolerance;
		} preconditionStruct;

		/**
		 * This class represents a state transition of the mission control system
		 * it is modelled as a Petri net transition having muptiple preconditions that \
		 * activate the transition once all of them are fulfilled
		 */
		class MissionStateTransition
		{
		public:
			/*
			 * Constructor taking the list of preconditions and the name of the state to go to after preconditions are met
			 *
			 * \param reader smart pointer to reader, should already be set to the appropriate node containing states
			 */
			MissionStateTransition(const labust::xml::ReaderPtr reader);

			/*
			 * Function to call when the Petri net receives a new symbol
			 * Processes the symbol and if it is a precondition marks it as met
			 * checks if the transition can be activated
			 *
			 * \param symbol received symbol, parameter name
			 * \param symbol received symbol, parameter value
			 * \returns true if all preconditions are met, false otherwise
			 */
			bool ProcessSymbol(const std::string &paramName, const std::string &paramValue);

			/*
			 * getter function for the state of the transition
			 *
			 * \returns state name
			 */
			std::string GetState();
			
		private:

			/* 
			 * Resets all preconditions to false, called when transition is triggerable
			 */
			void Reset();

			std::vector<preconditionStruct> transitionPreconditions;
			std::string state;
		};

		class MissionState
		{
		public:
			/*
			 * Constructor taking a preloaded XML file to read gonfig from and name of state to load
			 *
			 * \param reader smart pointer to reader, should already be set to the appropriate node containing states
			 * \param stateName name of state to load
			 */
			MissionState(const labust::xml::ReaderPtr reader, const std::string &stateName);

			/*
			 * Function to call when the Petri net receives a new symbol
			 * checks the symbol against all transitions
			 * returns name of new state if a transition is activated
			 *
			 * \param symbol received symbol, parameter name
			 * \param symbol received symbol, parameter value
			 * \returns name of new state if all conditions are met, empty string otherwise
			 */
			std::string ProcessSymbol(const std::string &paramName, const std::string &paramValue);

			/*
			 * Getter function for output of the state
			 */
			std::map<std::string,std::string> GetOutput();

		private:

			std::vector<MissionStateTransition> transitions;
			std::map<std::string,std::string> output;
			std::string name;

		};

		/**
		 * This class implements a Petri net based high level mission controller
		 */
		class MissionControl
		{
		public:
			typedef boost::shared_ptr<MissionControl> Ptr;
			/**
			 * Main constructor. Takes a XML reader pointer and configures the mission controller.
			 *
			 * \param reader Pointer to the XMLReader object containing the configuration data.
			 * \param configToUse name of configuration to use, if empty, will use first one.
			 */
			MissionControl(const labust::xml::ReaderPtr reader, const std::string& configToUse = "");

			/**
			 * Main constructor. Takes a path to the config file and configures the mission controller.
			 *
			 * \param filePath path to xml config file.
			 * \param configToUse name of configuration to use, if empty, will use first one.
			 */
			MissionControl(const std::string& filePath, const std::string& configToUse = "");

			/*
			 * Destructor - responsible for stopping timer
			 */
			~MissionControl();

			/**
			 * Function used to process vehicle state as described in labust::vehicles::state namespace
			 * In state transitions, theese states are referenced as: NU.[whatever];
			 * ETA.[whatever], GEO.[whatever], LF.[whatever], SEN.[whatever]
			 *
			 * \param vehicleState stateMap containing vehicle state 
			 * \returns true if state was changed, false otherwise
			 */
			bool AddVehicleState(const labust::vehicles::stateMap& vehicleState);
			
			/**
			 * Function used to process generic data (string-string map)
			 *
			 * \param data string-string map containing data
			 * \returns true if state was changed, false otherwise
			 */
			bool AddOtherData(const std::map<std::string,std::string>& data);

			std::map<std::string,std::string> GetCurrentMissionCommand();
			
		private:
			/*
			 * routine common to both constructors to avoid boilerplating
			 */
			void LoadConfig(const labust::xml::ReaderPtr reader);

			void TimerThreadFunction();
			
			std::map<std::string,MissionState> allStates;
			std::map<std::string,MissionState>::iterator currentState;
			boost::mutex stateLocker, timerLocker;

			boost::thread timerThread;

			int timerSeconds;
			bool running;
		};
	}
}

/* MISSIONCONTROL_HPP_ */
#endif
