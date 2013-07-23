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
#ifndef HLCOMBINE_HPP_
#define HLCOMBINE_HPP_
#include <auv_msgs/BodyVelocityReq.h>

#include <boost/array.hpp>

#include <string>
#include <set>

struct ControllerInfo
{
	std::string name;
	std::vector<std::string> requiredSubControllers;
	std::vector<int> requiredDOF;
};



struct HLCombine
{
	HLCombine()
	{
		//Empty the usage case
		for (int i=0; i<nuVals.size(); ++i)	 nuVals[i]="";
	}

	typedef std::set<std::string> ControllerCollection;

	void setControllers(const std::vector<ControllerInfo>& controllers)
	{
		std::vector<std::string> disableControllers;
		std::vector<std::string> enableControllers;

		//enable all sub-controllers of the enabled controller
		std::vector<ControllerInfo> subcon;
		for (int j=0; j<controller.requiredSubControllers.size(); ++j)
		{
			//TEST IF REQUEIRED SUB-CONTROLLER IS REGISTERED
			subcon.push_back(regControllers[controller.requredSubControllers[j]]);
		}

		setControllers(subcon);

		//Enable basic controller requests
		for (int i=0; i< controllers.size(); ++i)
		{
			ControllerInfo& controller = controllers[i];
			for(int j=0; j<controller.requiredDOF.size(); ++j)
			{
				int reqDOF = controller.requiredDOF[j];
				if (nuVals[reqDOF] != "")
				{
					disableControllers.push_back(nuVals[reqDOF]);
					//schedule all sub-controllers of the disable controller for disable
					disableControllers.insert(disableControllers.end(),
							regControllers[nuVals[reqDOF]].requiredSubControllers.begin(),
							regControllers[nuVals[reqDOF]].requiredSubControllers.end());
				}
				nuVals[reqDOF] = controller.name;
			}

			enableControllers.push_back(controller.name);
		}
	}

	void step(const std::map<std::string, boost::array<double,6> >& results, boost::array<double,6>& nuRef)
	{
		for (size_t i=0; i<6;++i)
		{
			nuRef[0] = results[nuVals[0]][0];
		}
	}

	const std::map<std::string, ControllerInfo> regControllers;
	boost::array<std::string,6> nuVals;
};

#endif



