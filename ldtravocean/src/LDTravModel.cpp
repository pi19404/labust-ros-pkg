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
*  Created on: Feb 25, 2013
*  Author: Dula Nad
*********************************************************************/
#include <labust/navigation/LDTravModel.hpp>

#include <boost/numeric/ublas/banded.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

using namespace labust::navigation;

LDTravModel::LDTravModel():
		dvlNonLinear(true)
{
	this->initModel();
};

LDTravModel::~LDTravModel(){};

void LDTravModel::initModel()
{
  //std::cout<<"Init model."<<std::endl;
  x = vector::Zero(stateNum);
  xdot = 0;
  ydot = 0;
  //Setup the transition matrix
  derivativeAW();
  R0 = R;
  V0 = V;

  //std::cout<<"R:"<<R<<"\n"<<V<<std::endl;
}

void LDTravModel::calculateXYInovationVariance(const LDTravModel::matrix& P, double& xin,double &yin)
{
	xin = sqrt(P(xp,xp)) + sqrt(R0(xp,xp));
	yin = sqrt(P(yp,yp)) + sqrt(R0(yp,yp));
}

void LDTravModel::step(const input_type& input)
{
  x(u) += Ts*(-surge.Beta(x(u))/surge.alpha*x(u) + 1/surge.alpha * input(X));
  x(v) += Ts*(-sway.Beta(x(v))/sway.alpha*x(v) + 1/sway.alpha * input(Y));
  x(w) += Ts*(-heave.Beta(x(w))/heave.alpha*x(w) + 1/heave.alpha * (input(Z) + x(buoyancy)));
  x(r) += Ts*(-yaw.Beta(x(r))/yaw.alpha*x(r) + 1/yaw.alpha * input(N) + x(b));

  xdot = x(u)*cos(x(psi)) - x(v)*sin(x(psi)) + x(xc);
  ydot = x(u)*sin(x(psi)) + x(v)*cos(x(psi)) + x(yc);
  x(xp) += Ts * xdot;
  x(yp) += Ts * ydot;
  x(zp) += Ts * x(w);
  x(psi) += Ts * x(r);

  xk_1 = x;

  derivativeAW();
};

void LDTravModel::derivativeAW()
{
	A = matrix::Identity(stateNum, stateNum);

	A(u,u) = 1-Ts*(surge.beta + 2*surge.betaa*fabs(x(u)))/surge.alpha;
	A(v,v) = 1-Ts*(sway.beta + 2*sway.betaa*fabs(x(v)))/sway.alpha;
	A(w,w) = 1-Ts*(heave.beta + 2*heave.betaa*fabs(x(w)))/heave.alpha;
	A(w,buoyancy) = Ts/heave.alpha;
	A(r,r) = 1-Ts*(yaw.beta + 2*yaw.betaa*fabs(x(r)))/yaw.alpha;
	A(r,b) = Ts;

	A(xp,u) = Ts*cos(x(psi));
	A(xp,v) = -Ts*sin(x(psi));
	A(xp,psi) = Ts*(-x(u)*sin(x(psi) - x(v)*sin(x(psi))));
	A(xp,xc) = Ts;

	A(yp,u) = Ts*sin(x(psi));
	A(yp,v) = Ts*cos(x(psi));
	A(yp,psi) = Ts*(x(u)*cos(x(psi)) - x(v)*sin(x(psi)));
	A(yp,yc) = Ts;

	A(zp,w) = Ts;

	A(psi,r) = Ts;
}

const LDTravModel::output_type& LDTravModel::update(vector& measurements, vector& newMeas)
{
	std::vector<size_t> arrived;
	std::vector<double> dataVec;

	for (size_t i=0; i<newMeas.size(); ++i)
	{
		if (newMeas(i))
		{
			arrived.push_back(i);
			dataVec.push_back(measurements(i));
			newMeas(i) = 0;
		}
	}

	if (dvlNonLinear) derivativeH();

	measurement.resize(arrived.size());
	H = matrix::Zero(arrived.size(),stateNum);
	y = vector::Zero(arrived.size());
	R = matrix::Zero(arrived.size(),arrived.size());
	V = matrix::Zero(arrived.size(),arrived.size());

	for (size_t i=0; i<arrived.size();++i)
	{
		measurement(i) = dataVec[i];

		if (dvlNonLinear)
		{
			H.row(i)=Hnl.row(arrived[i]);
			y(i) = ynl(arrived[i]);
		}
		else
		{
			H(i,arrived[i]) = 1;
			y(i) = x(arrived[i]);
		}

		for (size_t j=0; j<arrived.size(); ++j)
		{
			R(i,j)=R0(arrived[i],arrived[j]);
			V(i,j)=V0(arrived[i],arrived[j]);
		}
	}

	//std::cout<<"Setup H:"<<H<<std::endl;
	//std::cout<<"Setup R:"<<R<<std::endl;
	//std::cout<<"Setup V:"<<V<<std::endl;

	return measurement;
}

void LDTravModel::estimate_y(output_type& y)
{
  y=this->y;
}

void LDTravModel::derivativeH()
{
	Hnl=matrix::Identity(stateNum,stateNum);
	ynl = Hnl*x;

	//Correct the nonlinear part
	ynl(u) = x(u)+x(xc)*cos(x(psi))+x(yc)*sin(x(psi));
	ynl(v) = x(v)-x(xc)*sin(x(psi))+x(yc)*cos(x(psi));

	//Correct for the nonlinear parts
	Hnl(u,xc) = cos(x(psi));
	Hnl(u,yc) = sin(x(psi));
	Hnl(v,xc) = -sin(x(psi));
	Hnl(v,yc) = cos(x(psi));
	Hnl(u,psi) = -x(xc)*sin(x(psi)) + x(yc)*cos(x(psi));
	Hnl(v,psi) = -x(xc)*cos(x(psi)) - x(yc)*sin(x(psi));
}

