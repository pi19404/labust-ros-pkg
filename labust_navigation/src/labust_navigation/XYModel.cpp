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
#include <labust/navigation/XYModel.hpp>
#include <vector>

using namespace labust::navigation;

XYModel::XYModel():
		xdot(0),
		ydot(0)
{
	this->initModel();
};

XYModel::~XYModel(){};

void XYModel::initModel()
{
  //std::cout<<"Init model."<<std::endl;
  x = vector::Zero(stateNum);
  xdot = 0;
  ydot = 0;
  //Setup the transition matrix
  derivativeAW();
  //std::cout<<"R:"<<R<<"\n"<<V<<std::endl;
}

const XYModel::output_type& XYModel::update(vector& measurements, vector& newMeas)
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

	measurement.resize(arrived.size());
	H = matrix::Zero(arrived.size(),stateNum);
	R = matrix::Zero(arrived.size(),arrived.size());
	V = matrix::Zero(arrived.size(),arrived.size());

	for (size_t i=0; i<arrived.size();++i)
	{
		measurement(i) = dataVec[i];
		H(i,arrived[i]) = 1;
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

//const XYModel::output_type& XYModel::yawUpdate(double yaw)
//{
//	derivativeHV(1);
//	measurement(psiOnly_m) = yaw;
//	return measurement;
//}
//
//const XYModel::output_type& XYModel::fullUpdate(double x,
//  		  double y,
//  		  double yaw)
//{
//	derivativeHV(3);
//	measurement(x_m) = x;
//	measurement(y_m) = y;
//	measurement(psi_m) = yaw;
//
//	return measurement;
//}
//
//const XYModel::output_type& XYModel::positionUpdate(double x,
//  		  double y)
//{
//	derivativeHV(2);
//	measurement(x_m) = x;
//	measurement(y_m) = y;
//
//	return measurement;
//}

void XYModel::calculateXYInovationVariance(const XYModel::matrix& P, double& xin,double &yin)
{
	xin = sqrt(P(xp,xp)) + sqrt(R0(xp,xp));
	yin = sqrt(P(yp,yp)) + sqrt(R0(yp,yp));
}

void XYModel::step(const input_type& input)
{
  x(u) += Ts*(-surge.Beta(x(u))/surge.alpha*x(u) + 1/surge.alpha * input(X));
  x(v) += Ts*(-sway.Beta(x(v))/sway.alpha*x(v) + 1/sway.alpha * input(Y));
  x(r) += Ts*(-yaw.Beta(x(r))/yaw.alpha*x(r) + 1/yaw.alpha * input(N) + 0*x(b2));

  xdot = x(u)*cos(x(psi)) - x(v)*sin(x(psi)) + x(xc);
  ydot = x(u)*sin(x(psi)) + x(v)*cos(x(psi)) + x(yc);
  x(xp) += Ts * xdot;
  x(yp) += Ts * ydot;
  x(psi) += Ts * (x(r) + 0*x(b1));

  xk_1 = x;

  derivativeAW();
};

void XYModel::derivativeAW()
{
	A = matrix::Identity(stateNum, stateNum);

	A(u,u) = 1-Ts*(surge.beta + 2*surge.betaa*fabs(x(u)))/surge.alpha;
	A(v,v) = 1-Ts*(sway.beta + 2*sway.betaa*fabs(x(v)))/sway.alpha;
	A(r,r) = 1-Ts*(yaw.beta + 2*yaw.betaa*fabs(x(r)))/yaw.alpha;
	A(r,b2) = 0*Ts;

	A(xp,u) = Ts*cos(x(psi));
	A(xp,v) = -Ts*sin(x(psi));
	A(xp,psi) = Ts*(-x(u)*sin(x(psi)) - x(v)*cos(x(psi)));
	A(xp,xc) = Ts;

	A(yp,u) = Ts*sin(x(psi));
	A(yp,v) = Ts*cos(x(psi));
	A(yp,psi) = Ts*(x(u)*cos(x(psi)) - x(v)*sin(x(psi)));
	A(yp,yc) = Ts;

	A(psi,r) = Ts;
	A(psi,b1) = 0*Ts;
}

//void XYModel::derivativeHV(int numMeas)
//{
//	H = mzeros(numMeas,stateNum);
//
//	if (numMeas == 1)
//	{
//		H(psiOnly_m,psi) = 1;
//		R = V = mzeros(1,1);
//		R(0,0) = R0(psi_m,psi_m);
//		V(0,0) = V0(psi_m,psi_m);
//	}
//	else if (numMeas == 2)
//	{
//		H(x_m,xp) = H(y_m,yp) = 1;
//		R = boost::numeric::ublas::subrange(R0, x_m,y_m, x_m,y_m);
//		V = boost::numeric::ublas::subrange(V0, x_m,y_m, x_m,y_m);
//	}
//	else
//	{
//		H(x_m,xp) = H(y_m,yp) = H(psi_m,psi) = 1;
//		R = R0;
//		V = V0;
//	}
//
//	measurement.resize(numMeas);
//}

void XYModel::estimate_y(output_type& y)
{
  y=H*x;
}

