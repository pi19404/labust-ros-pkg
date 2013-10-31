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
#include <labust/control/SOIdentification.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <boost/numeric/ublas/matrix_proxy.hpp>

using namespace labust::control;

SOIdentification::SOIdentification(
		double C, double X, double R, double E, double identLen):
		finished(false),
		relay(C,X),
		params(numParams,0),
		tSum(0),
		maxOsc(-std::numeric_limits<double>::max()),
		minOsc(std::numeric_limits<double>::max()),
		counterHigh(0),
		counterLow(0),
		eMaxError(E),
		eMinError(E),
		ref(0),
		identLen(identLen),
		xa_high(identLen,0),
		xa_low(identLen,0),
		e_min(identLen,0),
		e_max(identLen,0),
		t_min(identLen,0),
		t_max(identLen,0)
{this->reset();};

SOIdentification::~SOIdentification(){};

double SOIdentification::step(double error, double dT)
{
	//Calculate output
	double retVal = relay.step(error);
	//Update timebase.
	tSum += dT;

  //Preserve the minimum and maximum oscillation value.
  if (error > maxOsc) maxOsc=error;
  if (error < minOsc) minOsc=error;

  if (int sw = relay.hasSwitched())
  {
    //std::cout<<"Relay switched. Error:"<<error<<std::endl;

    //Remember response data.
    if (sw == 1)
    {
      xa_high[counterHigh] = error;
      e_min[counterLow] = minOsc;
      t_min[counterLow] = tSum;
      tSum = 0;
      counterLow=(counterLow+1)%identLen;
    }
    else
    {
      xa_low[counterLow] = error;
      e_max[counterHigh] = maxOsc;
      t_max[counterHigh] = tSum;
      tSum = 0;
      counterHigh=(counterHigh+1)%identLen;
    }

    //Reset oscillation min and max.
    maxOsc=-std::numeric_limits<double>::max();
    minOsc=std::numeric_limits<double>::max();

    //Get the parameters for this cycle.
    calculateParameters();
  }

  return retVal;
}

void SOIdentification::calculateParameters()
{
	using boost::numeric::ublas::row;

  double meanEMax(labust::math::mean(e_max)),
  		stdEMax(labust::math::std2(e_max)),

  		meanEMin(labust::math::mean(e_min)),
  		stdEMin(labust::math::std2(e_min)),

  		meanTMax(labust::math::mean(t_max)),
  		meanTMin(labust::math::mean(t_min)),

  		meanXaLow(labust::math::mean(xa_low)),
  		meanXaHigh(labust::math::mean(xa_high));

  double T(meanTMax+meanTMin),
  		Xm(0.5*(meanEMax-meanEMin)),
  		X0(0.5*(meanEMax+meanEMin)),
  		xa_star(0.5*(meanXaHigh-meanXaLow));

  double omega(2*M_PI/T),
  		C(relay.getAmplitude());

  double sq1((xa_star+X0)/Xm);
  sq1 = sqrt(1-sq1*sq1);
  double sq2((xa_star-X0)/Xm);
  sq2 = sqrt(1-sq2*sq2);

  params[alpha] = 2*C*(sq1+sq2)/(M_PI*omega*omega*Xm);
  params[kx]=(4.0*C*xa_star)/(omega*M_PI*Xm*Xm);
  params[kxx]=(3.0*C*xa_star)/(2*omega*omega*Xm*Xm*Xm);
  params[delta] = C*(meanTMax - meanTMin)/(meanTMax + meanTMin);
  params[wn] = omega;

  //std::cout<<"Xa_star"<<xa_star<<", X0"<<X0<<std::endl;
  //std::cout<<"Alpha = "<<params[alpha]<<", Kx = "<<params[kx]<<", Kxx = "<<params[kxx]<<", Delta:"<<params[delta]<<", w:"<<omega<<std::endl;

  finished = ((std::fabs(stdEMax/meanEMax)<eMaxError) && (std::fabs(stdEMin/meanEMin)<eMinError));
  minMaxError = (std::fabs(stdEMax/meanEMax) + std::fabs(stdEMin/meanEMin))/2;
}

void SOIdentification::reset()
{
	finished = false;
	xa_high = std::vector<double>(identLen,0);
	xa_low = std::vector<double>(identLen,0);
	e_min = std::vector<double>(identLen,0);
	e_max = std::vector<double>(identLen,0);
	t_min = std::vector<double>(identLen,0);
	t_max = std::vector<double>(identLen,0);
	tSum = 0;
	maxOsc = -std::numeric_limits<double>::max();
	minOsc = std::numeric_limits<double>::max();
	counterHigh = 0;
	counterLow = 0;
	relay.setAmplitude(relay.getAmplitude());
}
