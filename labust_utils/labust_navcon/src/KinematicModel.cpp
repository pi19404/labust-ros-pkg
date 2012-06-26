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

#include <boost/numeric/ublas/banded.hpp>

using namespace labust::navigation;

KinematicModel::KinematicModel()
{
	this->configure();
}

KinematicModel::~KinematicModel(){};

void KinematicModel::configure()
{
	Ts = 0.1;
	this->initModel();
};

void KinematicModel::initModel()
{
  //Setup the transition matrix
  A = eye(stateNum,stateNum);
  x = zeros(stateNum);

  A(xp,Vv) = Ts*std::cos(x(psi));
  A(xp,psi) = -Ts*x(Vv)*sin(x(psi));
  A(yp,Vv) = Ts*sin(x(psi));
  A(yp,psi) = Ts*x(Vv)*cos(x(psi));
  A(psi,r) = Ts;
  W = mzeros(stateNum,3);

  W(2,0) = 1;
  W(3,1) = 1;
  W(4,2) = 1;

  //These are the noise variances
  vector q(3);
  q(0) = std::pow(0.5,2);
  q(1) = std::pow(0.05,2);
  q(2) = std::pow(0.2,2);
  Q = boost::numeric::ublas::diagonal_matrix<double>(q.size(),q.data());
  H = eye(2,stateNum);
  V = eye(2,2);
  R = eye(2,2);
}

void KinematicModel::step(const input_type& input)
{
  //This model is already discrete and we update only the interesting parts
  x(xp) += Ts*x(Vv)*std::cos(x(psi));
  x(yp) += Ts*x(Vv)*std::sin(x(psi));
  x(psi) += Ts*x(r);

  //Linearize the matrix for KF
  A(xp,Vv) = Ts*std::cos(x(psi));
  A(xp,psi) = -Ts*x(Vv)*sin(x(psi));
  A(yp,Vv) = Ts*sin(x(psi));
  A(yp,psi) = Ts*x(Vv)*cos(x(psi));
  A(psi,r) = Ts;
};

void KinematicModel::estimate_y(output_type& y)
{
  y=prod(H,x);
}
