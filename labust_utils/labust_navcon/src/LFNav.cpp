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
#include <labust/navigation/LFNav.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/plugins/PlugableDefs.hpp>

using namespace labust::navigation;

LFNav::LFNav(const labust::xml::ReaderPtr reader, const std::string& id):
		stateM(nav.getState()),
	  covM(nav.getStateCovariance()),
		iteration(0),
		delayMeas(42),
		receiveFreq(21),
		T1(Nav::zeros(3)),
		T2(Nav::zeros(3))
{
	this->configure(reader,id);
}

void LFNav::configure(const labust::xml::ReaderPtr& reader, const std::string& id)
{
	_xmlNode* org_node = reader->currentNode();
	//reader->useNode(reader->value<_xmlNode*>("LFNav" + (id.empty()?"":("[@id='" + id + "']"))));

	nav.configure(reader);

	//Get the initial state after configuration.
	stateM = nav.getState();

	reader->useNode(org_node);
}

void LFNav::prediction(const labust::vehicles::tauMapRef tau)
{
	Nav::input_type tauIn(Nav::inputSize);
	tauIn(Nav::N) = tau[labust::vehicles::tau::N];
	tauIn(Nav::Z) = tau[labust::vehicles::tau::Z];
	tauIn(Nav::X) = tau[labust::vehicles::tau::X];

	nav.predict(tauIn);

	//Buffer the input vector
	tauM.push_back(tauIn);
	if (iteration > delayMeas) tauM.pop_front();
}

void LFNav::correction(const labust::vehicles::stateMapRef measurement, labust::vehicles::stateMapRef stateHat)
{
	using namespace labust::vehicles;
	labust::vehicles::stateMap::iterator end = measurement.end();

	bool basic((measurement.find(state::yaw) != end) && ((measurement.find(state::z) != end)));
	bool newMeasurement((measurement.find(state::x) != end) && ((measurement.find(state::y) != end)));

	if (basic && newMeasurement)
	{
		//recalculate the filter
		this->recalculate(measurement);
		std::cout<<"LFNAV: new measurement."<<std::endl;
	}
	else if (basic)
	{
		nav.correct(nav.measurement(measurement[state::yaw], measurement[state::z]));
	}

	//Added to avoid estimation of surge speed
	if (measurement.find(state::u) != end)
	{
		//Sets the surge speed to the measurement value if available.
		this->resetSurge(measurement[state::u]);
	}

	Nav::vectorref estimate=nav.getState();

	stateHat[state::yaw] = estimate(Nav::psi);
	stateHat[state::r] = estimate(Nav::r);
	stateHat[state::z] = estimate(Nav::z);
	stateHat[state::dH] = estimate(Nav::dH);
	stateHat[state::dV] = estimate(Nav::dV);
	stateHat[state::u] = estimate(Nav::u);

	if (basic || newMeasurement)
	{
		//Buffer the measurement vector
		Nav::vector meas(2);
		meas(0) = measurement[state::yaw];
		meas(1) = measurement[state::z];

		measM.push_back(meas);

		if (++iteration > delayMeas) measM.pop_front();

		//std::cout<<"State:"<<iteration<<std::endl;
	}
}

void LFNav::resetSurge(double u)
{
  Nav::vector st = nav.getState();
  st(Nav::u) = u;
  st(Nav::horzCurrent) = 0;
  st(Nav::vertCurrent) = 0;
  nav.setState(st);
}

/**
 * The method recalculates the filter when delayed measurements arrive.
 */
void LFNav::recalculate(const labust::vehicles::stateMapRef measurement)
{
	if (iteration >= delayMeas)
	{
		using namespace labust::vehicles;
		nav.setState(stateM);
		nav.setStateCovariance(covM);

		nav.predict(tauM[0]);
	  nav.correct(nav.measurement(measurement[state::yaw],
	  		measurement[state::x],
	  		measurement[state::y],
	  		measurement[state::z]));

    for (size_t i=1;i<delayMeas;++i)
    {
    	//Added to avoid estimation of surge speed
    	if (measurement.find(state::u) != measurement.end())
    	{
    		//Sets the surge speed to the measurement value if available.
    		this->resetSurge(measurement[state::u]);
    	}
      nav.predict(tauM[i]);
      nav.correct(nav.measurement(measM[i](0),measM[i](1)));

      if (i == receiveFreq)
      {
        stateM = nav.getState();
        covM = nav.getStateCovariance();
      }
    }

    /*
    if ((delayMeas < receiveFreq) && (iteration == (receiveFreq - delayMeas)))
    {
      stateM = nav.getState();
      covM = nav.getStateCovariance();
    }
    */

    tauM.erase(tauM.begin(),tauM.begin() +  receiveFreq);
    measM.erase(measM.begin(),measM.begin() + receiveFreq);

    iteration = delayMeas - receiveFreq;
	}
}

void LFNav::setCommand(const labust::apps::stringRef commands)
{
	try
	{
		this->unwrapFromXml(commands);
		//Provisional
		nav.setLine(T1,T2);
	}
	catch (std::exception& e){};

	std::cerr<<"LFNav:Line recalucated."<<std::endl;
}

void LFNav::getData(labust::apps::stringPtr data)
{
	(*data) = *this->wrapInXml();
};

PP_LABUST_MAKE_CLASS_XML_OPERATORS((labust)(navigation),LFNav,
		(LFModel::vector, T1)
		(LFModel::vector, T2))

#ifndef BUILD_WITHOUT_PLUGIN_HOOK
LABUST_EXTERN
{
	LABUST_EXPORT NavigationFactoryPtr createFactory()
  {
		return NavigationFactoryPtr(new NavigationFactory::Impl<LFNav>());
  }
};
#endif



