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
#include <labust/control/HDController.hpp>
#include <labust/xml/xmltools.hpp>
#include <labust/control/controltune.hpp>

using namespace labust::control;

HDController::HDController(const labust::xml::ReaderPtr reader)
{
	this->configure(reader);
}

void HDController::configure(const labust::xml::ReaderPtr reader)
{
	labust::vehicles::dataMap data;
	labust::xml::param2map(reader,&data,"param","@name");

	headingParams.alpha = data.at("alpha_r");
	headingParams.beta = data["beta_r"];
	headingParams.betaa = data["beta_rr"];
	headingParams.w = data.at("w_r");
	headingParams.max = data.at("Max_N");

	depthParams.alpha = data.at("alpha_w");
	depthParams.beta = data["beta_w"];
	depthParams.betaa = data["beta_ww"];
	depthParams.w = data.at("w_w");
	depthParams.max = data.at("Max_Z");

	labust::control::tuneController(headingParams, &heading);
	labust::control::tuneController(depthParams, &depth);
}

void HDController::getTAU(const labust::vehicles::stateMapRef stateRef,
					const labust::vehicles::stateMapRef state,
					const labust::vehicles::tauMapRef tau)
try
{
	using namespace labust::vehicles::tau;
	using namespace labust::vehicles::state;
	tau[N] = heading.step(stateRef.at(yaw),state.at(yaw));
	tau[Z] = depth.step(stateRef.at(z),state.at(z));
}
catch (std::exception& e)
{
	using namespace labust::vehicles::tau;
	tau[N] = tau[Z] = 0;
}

#ifndef BUILD_WITHOUT_PLUGIN_HOOK
LABUST_EXTERN
{
	LABUST_EXPORT ControlFactoryPtr createFactory()
  {
		return ControlFactoryPtr(new ControlFactory::Impl<HDController>());
  }
};
#endif

