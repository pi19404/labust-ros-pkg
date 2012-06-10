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
#include <labust/control/LFController.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/xmltools.hpp>

#include <cmath>
#include <map>

using namespace labust::control;

LFController::LFController(const labust::xml::ReaderPtr reader):
		useFL(false),
		tunned(false),
		beta_rr(0),
		aAngle(M_PI/4)
{
	this->configure(reader);
}

void LFController::setCommand(const labust::apps::stringRef cmd)
try
{
	/*
	labust::xml::ReaderPtr reader(new labust::xml::Reader(cmd));

	reader->useNode();

	if (reader.try_expression("tune"))
	{
		std::map<std::string, double> map;
		if (labust::xml::param2map(reader,"tune[@name='horizontal']","param","@name"))
		{
			this->tuneDH();
		};
	}
	*/
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
	std::cerr<<"Rejected command:"<<cmd<<std::endl;
}

void LFController::getData(const labust::apps::stringPtr data){};

void LFController::getTAU(const labust::vehicles::stateMapRef stateRef,
		const labust::vehicles::stateMapRef state, labust::vehicles::tauMapRef tau)
try
{
	using namespace labust::vehicles::tau;
	using namespace labust::vehicles::state;

	//Initialization. Handle this initialization better.
	/*if (init)
	{
		dh_controller.initialize(0,state[dH]);
		init = false;
	}*/

	this->adjustDH(state[u]);

	//state[dH] = 1;

	double dhout = dh_controller.step(stateRef.at(dH),state.at(dH));

	std::cout<<"Dh controller out:"<<dhout<<","<<stateRef[dH]<<","<<state[dH]<<std::endl;

	tau[N] = r_controller.step(dhout,state[r]);
			//(useFL)?beta_rr*state[r]*static_cast<double (*)(double)>(std::fabs)(state[r]):0);
	tau[Z] = dv_controller.step(stateRef[dV],state[dV]);
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
	using namespace labust::vehicles::tau;
	tau[X] = tau[N] = tau[Z] = 0;
}

void LFController::configure(const labust::xml::ReaderPtr& reader)
{
	 labust::xml::param2map(reader,&paramMap,"param","@name");
	 this->tunned = this->tuneController(paramMap);
}

bool LFController::tuneController(const labust::vehicles::dataMapRef data)
try
{
	//Copy parameters
	paramMap = data;
	labust::vehicles::dataMap::iterator it, end(data.end());

	//Configure the horizontal controller
	//Set I-P structure
	//this->r_controller.setStructure(0,1,0);

	double wh(data.at("w_h"));
	double ar(data.at("alpha_r"));
	double br(0);
	if ((it=data.find("beta_r")) != end){br = it->second;}
	else{beta_rr = data.at("beta_rr");};

	mfH[a4h] = 1/(wh*wh*wh*wh); mfH[a3h] = 4/(wh*wh*wh); mfH[a2h] = 6/(wh*wh); mfH[a1h] = 4/wh;

	//Set gains
	this->r_controller.setGains(mfH[a2h]/mfH[a4h]*ar, (mfH[a3h]/mfH[a4h]*ar - 0*br), 0,0);
	this->r_controller.setTs(data.at("Ts"));
	labust::math::Limit<double> limit(-data.at("Max_N"),data.at("Max_N"));
	this->r_controller.setLimits(limit);
	//The horizontal controller is tuned in real-time according to the surge speed.
	this->dh_controller.setTs(data.at("Ts"));

	limit.min = -0.5;
	limit.max = 0.5;
	this->dh_controller.setLimits(limit);

	if ((it=data.find("attack_angle")) != end) aAngle=M_PI*(it->second/180);

	//Configure the vertical controller
	//Set I-PD structure
	//this->dv_controller.setStructure(0,1,0);

  double wv(data.at("w_h"));
  mfV[a3v] = 1/(wv*wv*wv);
  mfV[a2v] = 3/(wv*wv);
  mfV[a1v] = 3/(wv);

	limit.min = -data.at("Max_Z");
	limit.max = data.at("Max_Z");
  this->dv_controller.setLimits(limit);
  this->dv_controller.setTs(data.at("Ts"));
  this->tuneController(data.at("Xi"));

  return this->tunned = true;
}
catch (std::exception& e)
{
	std::cerr<<"Exception during LFController tuning. Missing parameter? Error:"<<e.what()<<std::endl;
	return this->tunned = false;
}

bool LFController::tuneController(double Xi)
try
{
	paramMap["Xi"] = Xi;
	double aw(paramMap["alpha_w"]);
	double bw(paramMap["beta_w"]);
	this->dv_controller.setGains(aw*mfV[a1v]/(cos(Xi)*mfV[a3v]), //Kpv
			aw/(cos(Xi)*mfV[a3v]), //Kiv
			aw/cos(Xi)*mfV[a2v]/mfV[a3v] - bw/cos(Xi),0); //Kdv

	return this->tunned = true;
}
catch (std::exception& e)
{
	std::cerr<<"Exception during LFController tuning. Missing parameter? Error:"<<e.what()<<std::endl;
	return this->tunned = false;
}

void LFController::adjustDH(double surge)
{
	if (!surge) surge=0.1;
	surge = 0.1;

	double Kph = 1/(surge*mfH[a2h]);
	double Kdh = mfH[a1h]/(surge*mfH[a2h]);
	this->dh_controller.setGains(Kph,0, Kdh,0);
	double dsat((Kdh/Kph)*surge*sin(aAngle));

	std::cout<<"LFCONTROLLER:"<<Kph<<","<<Kdh<<","<<dsat<<std::endl;

	labust::math::Limit<double> limit(-Kph*dsat,Kph*dsat);
	this->dh_controller.setPLimits(limit);
}

#ifndef BUILD_WITHOUT_PLUGIN_HOOK
LABUST_EXTERN
{
	LABUST_EXPORT ControlFactoryPtr createFactory()
  {
		return ControlFactoryPtr(new ControlFactory::Impl<LFController>());
  }
};
#endif
