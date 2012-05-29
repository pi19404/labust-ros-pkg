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
#include <labust/vehicles/UUVApp.hpp>
#include <labust/plugins/PluginLoader.hpp>
#include <labust/vehicles/VehicleFactoryName.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/XMLWriter.hpp>
#include <labust/xml/xmltools.hpp>
#include <labust/comms/NavAidMsg.hpp>
#include <labust/xml/GyrosReader.hpp>
#include <labust/vehicles/VehicleDriver.hpp>

#include <boost/numeric/ublas/io.hpp>

#include <fstream>

using namespace labust::simulation;

template <class Array>
void init_array(Array& array)
{
  for (size_t i=0; i<array.size();++i) array[i]=0;
}

UUVApp::UUVApp(const labust::xml::ReaderPtr reader, const std::string& id):
				mode(idle),
				state(new labust::vehicles::stateMap()),
				measState(new labust::vehicles::stateMap()),
				targetDepth(10),
				xuuv(0),
				yuuv(0),
				newMeasurement(false),
				surgeForce(0),
				identificationStep(identNone),
				Ts(0.1),
				saveIdentification(false),
        tauM(LFNav::mzeros(100,LFNav::inputSize)),
        measM(LFNav::mzeros(100,2)),
        iterCnt(0),
        delayMeas(LFNav::mzeros(100,2))
{
	_xmlNode* org_node = reader->currentNode();
	reader->useNode(reader->value<_xmlNode*>("//UUVApp" + (id.empty()?"":("[@id='" + id + "']"))));

	//Basic configuration
	watchdog.reset(new labust::tools::watchdog(boost::bind(&UUVApp::stop,this),
			reader->value<size_t>("watchdog/@timeout"),
			reader->value<size_t>("watchdog/@loop")));

	surgeForce = reader->value<double>("OpenLoopTauX");
	//Configure controllers
	lfCon.reset(new labust::control::LFController(reader,"LineFollowing"));
	if (!lfCon->isTunned())
		throw std::invalid_argument("Line following controller not tunned. Missing parameter?");

	_xmlNode* cnode = reader->currentNode();
	reader->useNode(reader->value<_xmlNode*>("Controller[@id='LineFollowing']"));
	labust::vehicles::dataMap data;
	labust::xml::param2map(reader,&data,"param","@name");

	yawParam.alpha = data.at("alpha_r");
	yawParam.beta = data["beta_r"];
	yawParam.betaa = data["beta_rr"];
	yawParam.w = data.at("w_r");
	yawParam.max = data.at("Max_N");

	depthParam.alpha = data.at("alpha_w");
	depthParam.beta = data["beta_w"];
	depthParam.betaa = data["beta_ww"];
	depthParam.w = data.at("w_w");
	depthParam.max = data.at("Max_Z");

	this->tuneController(yawParam,&yawCon);
	this->tuneController(depthParam,&depthCon);

	yawCon.setStructure(0,1,0);
	depthCon.setStructure(0,1,0);

	//Configure navigation
	nav.reset(new LFNav());
	//nav.reset(new LFNav(reader));
	nav->configure(reader);

	nav2 = new labust::navigation::LFNav(reader,"");

	nav->setStateCovariance(1000*LFNav::eye(LFNav::stateNum));

  iterCnt = 0;
  stateM = nav->getState();
  convP = nav->getStateCovariance();

	reader->useNode(cnode);

	//Try the identification save flag
	reader->try_value("saveIdentification",&saveIdentification);

	//Configure identification
	headingSOId.first = reader->value<double>("heading-so/@tau");
	headingSOId.second = reader->value<double>("heading-so/@angle") * M_PI/180;

	depthSOId.first = reader->value<double>("depth-so/@tau");
	depthSOId.second = reader->value<double>("depth-so/@depth");

	//Configure vehicle driver
	plugin.reset(
			new labust::vehicles::VehiclePlugin(reader->value<std::string>("plugin/@name"),
					labust::vehicles::FactoryCreatorName::value));
	uuv.reset(labust::plugins::createConfiguredInstance(reader, *plugin));

	reader->useNode(org_node);

	//\todo Check initalization with list on g++4.3 and 10.04.
	//this->tau={{0}};
	init_array(this->tau);
	//this->tauk_1={{0}};
	init_array(this->tauk_1);
}

const labust::vehicles::stateMapRef UUVApp::step()
{
	using namespace labust::vehicles::state;

	//Get current state measurements
	uuv->getState(measState);

	//Prepare prediction step use Tauk_1
//  LFNav::input_type tauIn(LFNav::inputSize);
//	tauIn(LFNav::N) = tauk_1[labust::vehicles::tau::N];
//	tauIn(LFNav::Z) = tauk_1[labust::vehicles::tau::Z];
//	tauIn(LFNav::X) = tauk_1[labust::vehicles::tau::X];
//
//  boost::numeric::ublas::row(tauM,iterCnt%100) = tauIn;
//  LFNav::vector meas(2);
//  measM(iterCnt,0) = (*measState)[yaw];
//  measM(iterCnt,1) = (*measState)[z];
//
//  delayMeas(iterCnt,0) = (*measState)[x];
//  delayMeas(iterCnt,1) = (*measState)[y];
//
//  iterCnt = (++iterCnt)%100;

  nav2->prediction(tauk_1);

//	nav->predict(tauIn);
//
//	std::cout<<"Trace of P:"<<nav->traceP()<<std::endl;

	enum {measDelay = 42, measDiff=21};

	//Make correction step
	if (newMeasurement /*&& iterCnt >= measDelay*/)
	{
//		std::cout<<"Correcting:"<<xuuv<<","<<yuuv<<","<<(*measState)[x]<<","<<(*measState)[y]<<std::endl;
//    std::cout<<"Iteration count:"<<iterCnt<<std::endl;
//    //std::cout<<"Real position:"<<(*measState)[x]<<","<<(*measState)[y]<<std::endl;
//    //Using real states for KF updates
//    if (iterCnt >= measDelay)
//    {
//    nav->setState(stateM);
//    nav->setStateCovariance(convP);
//  	if (mode == lineFollowing)
//  	{
//      LFNav::vector st = nav->getState();
//  	  st(LFNav::u) = 0.1;
//  	  st(LFNav::horzCurrent) = 0;
//  	  st(LFNav::vertCurrent) = 0;
//  	  nav->setState(st);
//    }
//    nav->predict(boost::numeric::ublas::row(tauM,0));
//    //nav->correct(nav->measurement((*measState)[yaw], xuuv, yuuv,(*measState)[z]));
//    nav->correct(nav->measurement(measM(0,0), xuuv, yuuv, measM(0,1)));
//    //nav->correct(nav->measurement(measM(0,0), delayMeas(0,0), delayMeas(0,1),measM(0,1)));
//
//    for (int i=1;i<iterCnt;++i)
//    {
//    	if (mode == lineFollowing)
//    	{
//        LFNav::vector st = nav->getState();
//    	  st(LFNav::u) = 0.1;
//    	  st(LFNav::horzCurrent) = 0;
//    	  st(LFNav::vertCurrent) = 0;
//    	  nav->setState(st);
//      }
//      nav->predict(boost::numeric::ublas::row(tauM,i));
//      nav->correct(nav->measurement(measM(i,0),measM(i,1)));
//
//      //Changed from iterCnt==measDiff
//      if (i == measDiff)
//      {
//        stateM = nav->getState();
//        convP = nav->getStateCovariance();
//      }

      labust::vehicles::stateMap measurement = *measState;
      if (mode == lineFollowing) measurement[u] = 0.1;
      measurement[x] = xuuv;
      measurement[y] = yuuv;

      nav2->correction(measurement, state);
//    }
//
//    std::cout<<"Tau first:"<<tauM(21,1)<<std::endl;
//
//    boost::numeric::ublas::subrange(tauM, 0,measDelay - measDiff, 0,2) = boost::numeric::ublas::subrange(tauM, measDiff,measDelay, 0,2);
//    boost::numeric::ublas::subrange(measM, 0,measDelay - measDiff, 0,2) = boost::numeric::ublas::subrange(measM, measDiff,measDelay, 0,2);
//
//    std::cout<<"Tau second:"<<tauM(0,1)<<std::endl;
//
//
//		//std::cout<<"Real position:"<<(*measState)[x]<<","<<(*measState)[y]<<std::endl;
//		//Using real states for KF updates
//		//nav->correct(nav->measurement((*measState)[yaw], xuuv, yuuv,(*measState)[z]));
//		//Self-loop on line-following - using the real state for KF updates
//		//nav->correct(nav->measurement((*measState)[yaw], (*measState)[x], (*measState)[y],(*measState)[z]));
//
//      //Changed form =measDiff
//      //iterCnt = measDelay - measDiff;
//      iterCnt = measDelay - measDiff;
//      //stateM = nav->getState();
//      //convP = nav->getStateCovariance();
//  		//nav->correct(nav->measurement((*measState)[yaw], delayMeas(0,0), delayMeas(0,1),(*measState)[z]));
  		newMeasurement = false;
//    }
	}
	else
	{
    labust::vehicles::stateMap measurement = *measState;
    measurement.erase(x);
    measurement.erase(y);
    if (mode == lineFollowing) measurement[u] = 0.1;
		nav2->correction(measurement, state);
		//nav->correct(nav->measurement((*measState)[yaw],(*measState)[z]));
	}

	//Force surge to 0.1.
	//Enable this when using real tests.
//	if (mode == lineFollowing)
//	{
//    LFNav::vector st = nav->getState();
//	  st(LFNav::u) = 0.1;
//	  st(LFNav::horzCurrent) = 0;
//	  st(LFNav::vertCurrent) = 0;
//	  nav->setState(st);
//  }
//	else
//	{
//    LFNav::vector st = nav->getState();
//	  st(LFNav::u) = 0;
//	  st(LFNav::horzCurrent) = 0;
//	  st(LFNav::vertCurrent) = 0;
//	  nav->setState(st);
//	}
//
//	const LFNav::Line& line = nav->getLine();
//
//	//std::cout<<"Vehicle true state:"<<state->at(yaw)<<","<<state->at(z)<<std::endl;
//	std::cout<<"Vehicle true state:"<<line.calculatedH((*measState)[x],(*measState)[y],(*measState)[z])<<", "<<(*state)[u]<<std::endl;
	//std::cout<<"Vehicle estimate:"<<(nav->getState()(LFNav::dH))<<", u="<<(nav->getState()(LFNav::u))<<std::endl;
	std::cout<<"Vehicle estimate:"<<(*state)[dH]<<", u="<<(*state)[dV]<<std::endl;
	std::cout<<"Vehicle estimate full:"<<(*state)[yaw]<<","<<(*state)[z]<<","<<(*state)[u]<<std::endl;
	std::cout<<"Vehicle heading,depth,speed:"<<(*measState)[yaw]<<","<<(*measState)[z]<<","<<(*measState)[u]<<std::endl;

	//Update the state with KF estimates.
	/*(*state)[yaw] = nav->getState()(LFNav::psi);
	(*state)[r] = nav->getState()(LFNav::r);
	(*state)[z] = nav->getState()(LFNav::z);
	(*state)[dH] = nav->getState()(LFNav::dH);
	(*state)[dV] = nav->getState()(LFNav::dV);
	(*state)[u] = nav->getState()(LFNav::u);*/

	//Get next control
	std::cout<<"TauX:"<<tau[0]<<",TauN:"<<tau[5]<<",TauZ:"<<tau[2]<<", Mode:"<<mode<<std::endl;
	calculateTau();

	uuv->setTAU(tau);
	tauk_1 = tau;

	return *state;
}

void UUVApp::stop()
{
	mode = idle;
	//tau={{0}};
	init_array(tau);
}

void UUVApp::calculateTau()
{
	switch (mode)
	{
	case idle:
		//tau={{0}};
		init_array(tau);
		break;
	case identification:
		this->doIdentification();
		break;
	case manual: break;
	case manualControl:
		using namespace labust::vehicles::tau;
		using namespace labust::vehicles::state;
		tau[N] = yawCon.step(stateRef[yaw],state->at(yaw));
		tau[Z] = depthCon.step(stateRef[z],state->at(z));
		break;
	case lineFollowing:
		using namespace labust::vehicles::tau;
		using namespace labust::vehicles::state;
		tau[X] = surgeForce;
		lfCon->getTAU(stateRef,*state,&tau);
		break;
	default:
		init_array(tau);
		//tau={{0}};
		break;
	}

	return;
}

void UUVApp::nextIdentificationStep()
{
	switch (identificationStep)
	{
	case identHeading:
		identificationStep = identFinished;

		soIdent.reset();
		soIdent.setRelay(depthSOId.first,depthSOId.second);
		stateRef[labust::vehicles::state::z] = 2*depthSOId.second;
		break;
	case identDepth:
		identificationStep = identFinished;

		if (saveIdentification)
		{
			labust::xml::Writer writer;
			writer.startDocument();
			 writer.startElement("SO-I");
			  writer.startElement("yaw");
			   writer.addAttribute("alpha",yawParam.alpha);
			   writer.addAttribute("beta",yawParam.beta);
			   writer.addAttribute("betaa",yawParam.betaa);
			  writer.endElement();
			  writer.startElement("depth");
			   writer.addAttribute("alpha",yawParam.alpha);
			   writer.addAttribute("beta",yawParam.beta);
			   writer.addAttribute("betaa",yawParam.betaa);
			writer.endDocument();

			std::fstream file("SO-I_results_" + labust::tools::time_signature());
			file<<writer.toString();
			file.flush();
		}

		break;
	case identFinished:
	default:
		mode = idle;
		break;
	}
}

void UUVApp::doIdentification()
{
	switch (identificationStep)
	{
	case identHeading:
		using namespace labust::vehicles::tau;
		using namespace labust::vehicles::state;
		if (!soIdent.isFinished())
		{
			init_array(tau);
			//tau = {{0}};
			tau[N] = soIdent.step(stateRef[yaw] - measState->at(yaw),Ts);
		}
		else
		{
			labust::control::SOIdentification::ParameterContainer param;
			soIdent.parameters(&param);

			yawParam.alpha = param[labust::control::SOIdentification::alpha];
			yawParam.beta = param[labust::control::SOIdentification::kx];
			yawParam.betaa = param[labust::control::SOIdentification::kxx];

			this->tuneController(yawParam,&yawCon);
			this->nextIdentificationStep();
		}
		break;
	case identDepth:
		using namespace labust::vehicles::tau;
		using namespace labust::vehicles::state;
		if (!soIdent.isFinished())
		{
			//tau = {{0}};
			init_array(tau);
			tau[Z] = soIdent.step(stateRef[z] - state->at(z),Ts);
		}
		else
		{
			labust::control::SOIdentification::ParameterContainer param;
			soIdent.parameters(&param);

			depthParam.alpha = param[labust::control::SOIdentification::alpha];
			depthParam.beta = param[labust::control::SOIdentification::kx];
			depthParam.betaa = param[labust::control::SOIdentification::kxx];

			this->tuneController(depthParam,&depthCon);

			this->nextIdentificationStep();
		}
		break;
	case identFinished:
	default:
		this->nextIdentificationStep();
		break;
	}
}

void UUVApp::newModemData(const labust::xml::GyrosReader& reader)
try
{
	watchdog->reset();
	const std::string& label = reader.GetLabel();

	if (label.compare(labust::comms::NavAidMsg::Manual) == 0)
	{
		mode = manual;
		std::map<std::string,double> values;
		reader.dictionary(values);
		using namespace labust::vehicles::tau;
		tau[X] = values.at("X");
		tau[Z] = values.at("Z");
		tau[N] = values.at("N");
	}
	else if (label.compare(labust::comms::NavAidMsg::Identification)  == 0)
	{
		mode = identification;
		soIdent.reset();
	}
	else if (label.compare(labust::comms::NavAidMsg::ManualControl) == 0)
	{
		mode = manualControl;
		std::map<std::string,double> values;
		reader.dictionary(values);
		using namespace labust::vehicles::tau;
		using namespace labust::vehicles::state;
		tau[X] = values["X"];
		stateRef[yaw] = labust::math::wrapRad(values["Heading"]/180*M_PI);
		stateRef[z] = values["Depth"];
	}
	else if (label.compare(labust::comms::NavAidMsg::LineFollowing) == 0)
	{
		bool reinit(mode != lineFollowing);
		mode = lineFollowing;
		std::map<std::string,double> values;
		reader.dictionary(values);
		//extract new measurements
		extractMeasurements(values);
		//extractMeasurements2(values);

		//recalculate line if needed (on mode change)
		if (reinit)
		{
			labust::navigation::LFModel::vector T2(labust::navigation::LFModel::zeros(3)),T1(3);
			T2(2) = targetDepth;

			//Enable to use measurements
			T1(0) = xuuv;
			T1(1) = yuuv;

			//Enable self-loop on line following - use real data for LF
			//T1(0) = (*measState)[labust::vehicles::state::x];
			//T1(1) = (*measState)[labust::vehicles::state::y];
			//T1(2) = 0;
			T1(2) = (*measState)[labust::vehicles::state::z];
			//Enable self-loop on line following
			nav->setLine(T1,T2);

			labust::vehicles::dataMap cmd;
			cmd["TargetDepth"] = targetDepth;
			cmd["xUUV"] = xuuv;
			cmd["yUUV"] = yuuv;

			nav2->setCommand(cmd);

      iterCnt = 0;
      stateM = nav->getState();
      convP = nav->getStateCovariance();
		}
	}
	else if (label.compare(labust::comms::NavAidMsg::Idle) == 0)
	{
		mode = idle;
		std::map<std::string,double> values;
		reader.dictionary(values);
		//extract new measurements
		extractMeasurements(values);
	}
	else if (label.compare(labust::comms::NavAidMsg::SetupTarget) == 0)
	{
		std::map<std::string,double> values;
		reader.dictionary(values);
		targetDepth = values["z"];
		//Enable to have regular setup target
		mode = idle;
		//Enable to have identification
		//mode = identification;
		if (identificationStep == identNone)
		{
			identificationStep = identHeading;
			soIdent.reset();
			soIdent.setRelay(headingSOId.first,headingSOId.second);
			using namespace labust::vehicles::state;
			stateRef[yaw] = (*measState)[yaw];
		}
	}
	else
	{
		mode = idle;
	}
}
catch (std::exception& e)
{
	std::cerr<<e.what();
	//tau = {{0}};
	init_array(tau);
}

void UUVApp::extractMeasurements(std::map<std::string, double>& values)
{
	double rh(std::pow(values["range"],2));
	rh -= std::pow((*state)[labust::vehicles::state::z],2);

	std::cout<<"Measurement:"<<values["range"]<<", bearing:"<<values["bearing"]<<std::endl;

	double bearing = labust::math::wrapRad(M_PI*values["bearing"]/180);
	xuuv = values["x"] + std::cos(bearing)*std::sqrt(std::fabs(rh));
	yuuv = values["y"] + std::sin(bearing)*std::sqrt(std::fabs(rh));

	//(*measState)[labust::vehicles::state::x] = xuuv;
	//(*measState)[labust::vehicles::state::y] = yuuv;

	this->newMeasurement = true;
}

void UUVApp::extractMeasurements2(std::map<std::string, double>& values)
{
	double rh(std::pow(this->range,2));
	rh -= std::pow((*state)[labust::vehicles::state::z],2);

	double bearing = labust::math::wrapRad(M_PI*this->bearing/180);
	xuuv = this->usvx + std::cos(bearing)*std::sqrt(std::fabs(rh));
	yuuv = this->usvy + std::sin(bearing)*std::sqrt(std::fabs(rh));

	//(*measState)[labust::vehicles::state::x] = xuuv;
	//(*measState)[labust::vehicles::state::y] = yuuv;

	this->newMeasurement = true;
}

void UUVApp::setCurrents(const boost::array<double,3>& current)
{
	labust::vehicles::dataMap data;
	enum {current_x,current_y,current_z};

	data["Current_X"] = current[current_x];
	data["Current_Y"] = current[current_y];
	data["Current_Z"] = current[current_z];
	uuv->setCommand(data);
}
