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
#include <labust/navigation/KinematicModel.hpp>

using namespace labust::navigation;

KinematicModel::KinematicModel(){this->initModel();};

KinematicModel::~KinematicModel(){};

void KinematicModel::initModel()
{
  x = Base::vector::Zero(stateNum);
  //Setup the transition matrix
  A = Base::matrix::Identity(stateNum,stateNum);

  A(xp,Vv) = Ts*std::cos(x(psi));
  A(yp,Vv) = Ts*std::sin(x(psi));
  A(psi,r) = Ts;
  A(r,r) = 0.9;
  W = Base::matrix::Identity(stateNum,stateNum);

  //These are the noise variances
  vector q(stateNum);
  q<<std::pow(0.02,2), //x
  std::pow(0.02,2), //y
  std::pow(0.07,2), //u
  std::pow(0.01,2), //psi
  std::pow(0.1,2); //r
  Q = q.asDiagonal();
  H = Base::matrix::Identity(inputSize,stateNum);
  V = Base::matrix::Identity(inputSize,inputSize);
  R = 16*Base::matrix::Identity(inputSize,inputSize);
}

void KinematicModel::step(const input_type& input)
{
  //This model is already discrete and we update only the interesting parts
  A(xp,Vv) = Ts*std::cos(x(psi));
  A(yp,Vv) = Ts*std::sin(x(psi));
  A(xp,psi) = 0;
  A(yp,psi) = 0;

  //Propagate state
  //x(Vv) = 0.5;
  //x(r) = 0;
  x = A*x;
  //x(r) = 0;
  //x(Vv) = 0.5;

  //Linearize the matrix for
  A(xp,psi) = -Ts*x(Vv)*sin(x(psi));
  A(yp,psi) = Ts*x(Vv)*cos(x(psi));
};

void KinematicModel::estimate_y(output_type& y)
{
  y=H*x;
}

void KinematicModel::calculateXYInovationVariance(const KinematicModel::matrix& P,
		double& xin,double &yin)
{
	xin = sqrt(P(xp,xp)) + sqrt(R(xp,xp));
	yin = sqrt(P(yp,yp)) + sqrt(R(yp,yp));
}
