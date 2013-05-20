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

LDTravModel::LDTravModel()
{
	this->initModel();
};

LDTravModel::~LDTravModel(){};

void LDTravModel::initModel()
{
  //std::cout<<"Init model."<<std::endl;
  x = zeros(stateNum);
  xdot = 0;
  ydot = 0;
  //Setup the transition matrix
  derivativeAW();
  R0 = R;
  V0 = V;

  std::cout<<"R:"<<R<<"\n"<<V<<std::endl;
}

//const LDTravModel::output_type& LDTravModel::yawUpdate(double yaw)
//{
//	derivativeHV(1);
//	measurement(psiOnly_m) = yaw;
//	return measurement;
//}
//
//const LDTravModel::output_type& LDTravModel::yawSpeedUpdate(double u,
//		double v,
//		double w,
//		double yaw)
//{
//	derivativeHV(4);
//	measurement(upsi_m) = u;
//	measurement(wpsi_m) = v;
//	measurement(wpsi_m) = w;
//	measurement(psipsi_m) = yaw;
//	return measurement;
//}

//const LDTravModel::output_type& LDTravModel::fullUpdate(double u,
//				double v,
//				double w,
//				double x,
//  		  double y,
//  		  double yaw)
//{
//	derivativeHV(6);
//	measurement(u_m) = u;
//	measurement(v_m) = v;
//	measurement(w_m) = w;
//	measurement(x_m) = x;
//	measurement(y_m) = y;
//	measurement(psi_m) = yaw;
//
//	return measurement;
//}

//const LDTravModel::output_type& LDTravModel::yawPositionUpdate(double x,
//  		  double y, double yaw)
//{
//	derivativeHV(3);
//	measurement(xpsi_m) = x;
//	measurement(ypsi_m) = y;
//	measurement(xypsipsi_m) = yaw;
//
//	return measurement;
//}

void LDTravModel::calculateXYInovationVariance(const LDTravModel::matrix& P, double& xin,double &yin)
{
	xin = sqrt(P(xp,xp)) + sqrt(R0(x_m,x_m));
	yin = sqrt(P(yp,yp)) + sqrt(R0(y_m,y_m));
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
  x(w) += Ts * x(w);
  x(psi) += Ts * x(r);

  xk_1 = x;

  derivativeAW();
};

void LDTravModel::derivativeAW()
{
	A = eye(stateNum);

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

	R = mzeros(arrived.size(),arrived.size());
	V = mzeros(arrived.size(),arrived.size());

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

	return measurement;
}

//void LDTravModel::derivativeHV(int numMeas)
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
//	else if (numMeas == 3)
//	{
//		using namespace boost::numeric::ublas;
//		H(xpsi_m,xp) = H(ypsi_m,yp) = H(wpsi_m,w) = H(psipsi_m,psi) = 1;
//		R = mzeros(numMeas,numMeas);
//		V = mzeros(numMeas,numMeas);
//		subrange(R, u_m,w_m, u_m,w_m) = subrange(R0, u_m,w_m, u_m,w_m);
//		subrange(V, u_m,w_m, u_m,w_m) = subrange(V0, u_m,w_m, u_m,w_m);
//		R(psipsi_m, u_m) = R0(psi_m,u_m);
//		R(psipsi_m, v_m) = R0(psi_m,v_m);
//		R(psipsi_m, w_m) = R0(psi_m,w_m);
//		R(psipsi_m, psipsi_m) = R0(psi_m,psi_m);
//	}
//	else if (numMeas == 4)
//	{
//		using namespace boost::numeric::ublas;
//		H(upsi_m,u) = H(vpsi_m,v) = H(wpsi_m,w) = H(psipsi_m,psi) = 1;
//		R = mzeros(numMeas,numMeas);
//		V = mzeros(numMeas,numMeas);
//		subrange(R, u_m,w_m, u_m,w_m) = subrange(R0, u_m,w_m, u_m,w_m);
//		subrange(V, u_m,w_m, u_m,w_m) = subrange(V0, u_m,w_m, u_m,w_m);
//		R(psipsi_m, u_m) = R0(psi_m,u_m);
//		R(psipsi_m, v_m) = R0(psi_m,v_m);
//		R(psipsi_m, w_m) = R0(psi_m,w_m);
//		R(psipsi_m, psipsi_m) = R0(psi_m,psi_m);
//	}
//	else
//	{
//		H(upsi_m,u) = H(vpsi_m,v) = H(wpsi_m,w) = 1;
//		H(x_m,xp) = H(y_m,yp) = H(psi_m,psi) = 1;
//		R = R0;
//		V = V0;
//	}
//
//	measurement.resize(numMeas);
//}

void LDTravModel::estimate_y(output_type& y)
{
  y=prod(H,x);
}

