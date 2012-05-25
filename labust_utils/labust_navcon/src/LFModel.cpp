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
#include <labust/navigation/LFModel.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/xml/XMLuBlas.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/banded.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace labust::navigation;

LFModel::LFModel()
{
  this->initModel();
}

LFModel::LFModel(const labust::xml::ReaderPtr reader)
{
  this->configure(reader);
}

const LFModel::output_type& LFModel::measurement(double yaw, double depth)
{
  //Define the output matrix
  derivativeHV(2);
  meas(z_m) = depth;
  meas(psi_m) = yaw;
  return meas;
}

const LFModel::output_type& LFModel::measurement(double yaw, double x0, double y0, double z0)
{
  derivativeHV(4);
  meas(z_m) = z0;
  meas(psi_m) = yaw;
  meas(dH_m) = line.calculatedH(x0,y0,z0);
  meas(dV_m) = line.calculatedV(x0,y0,z0);
  return meas;
}

double LFModel::Line::calculatedH(double x0,double y0,double z0) const
{
  double Lp = sqrt((T2(xp)-T1(xp))*(T2(xp)-T1(xp))+(T2(yp)-T1(yp))*(T2(yp)-T1(yp)));
  //Note the minus thingy
  return -((T2(xp)-T1(xp))*(T1(yp)-y0)-(T1(xp)-x0)*(T2(yp)-T1(yp)))/Lp;
}

double LFModel::Line::calculatedV(double x0, double y0, double z0) const
{
  double Lp = sqrt(std::pow(T2(xp)-T1(xp),2) + std::pow(T2(yp)-T1(yp),2));
  double L = sqrt(Lp*Lp + std::pow(T2(zp) - T1(zp),2));
  double n = (T2(xp)-T1(xp))*(x0-T1(xp))+(T2(yp)-T1(yp))*(y0-T1(yp));
  n *= -(T2(zp)-T1(zp));
  n -= (T1(zp)-z0)*Lp*Lp;

  return n/(Lp*L);
}

void LFModel::Line::setLine(const LFModel::vector& T1, const LFModel::vector& T2)
{
  this->T1 = T1;
  this->T2 = T2;

  this->Gamma = atan2(T2(yp) - T1(yp),T2(xp) - T1(xp));
  this->Xi = atan2(T2(zp) - T1(zp),sqrt(std::pow(T2(xp)-T1(xp),2) + std::pow(T2(yp)-T1(yp),2)));
}

void LFModel::step(const LFModel::input_type& input)
{
  //Calculate the matrices A,W
  derivativeAW();
  //Progress the model one step forward.
  //Note: Although we have numerically some estimates from time step "K" available
  //the theory requires us to use measurements from time step "K-1"

  x(w) += Ts*(-betaW/alphaW*xk_1(w) + 1/alphaW * (input(Z) + x(buoyancy)));
  x(r) += Ts*(-betaR/alphaR*xk_1(r) + 1/alphaR * input(N));
  x(psi) += Ts*xk_1(r);
  x(z) += Ts*xk_1(w);
  x(dV) += Ts * (xk_1(w)*cos(line.xi()) - xk_1(u)*cos(xk_1(psi) - line.gamma())*sin(line.xi()) + xk_1(vertCurrent));
  x(dH) += Ts * (xk_1(u)*sin(xk_1(psi) - line.gamma()) + xk_1(horzCurrent));
  //rest of values are constant
  xk_1 = x;
}

void LFModel::derivativeAW()
{
  double sg = sin(x(psi) - line.gamma());
  double cg = cos(x(psi) - line.gamma());

  //Change only variable parts
  A(dV,w) = Ts * cos(line.xi());
  A(dV,u) = Ts * (-cg * sin(line.xi()));
  A(dV,psi) = Ts * x(u) * sg * sin(line.xi());
  A(dH,u) = Ts * sg;
  A(dH,psi) = Ts * x(u) * cg;
}

void LFModel::derivativeHV(int numMeas)
{
  H = mzeros(numMeas,stateNum);
  H(z_m,z) = H(psi_m,psi) = 1;

  if (numMeas == 4)
  {
    V=V0;
    R=R0;
    H(dV_m,dV) = H(dH_m,dH) = 1;
  }
  else
  {
    //Its enough to just change V
    R=boost::numeric::ublas::subrange(R0, 0,2, 0,2);
    V=boost::numeric::ublas::subrange(V0, 0,2, 0,2);
  }

  meas.resize(numMeas);
}

void LFModel::estimate_y(output_type& y)
{
  //Acutally hk(x) is the same as H*x in this model case.
  y = prod(H,x);
}

void LFModel::configure(const labust::xml::ReaderPtr reader)
{
  reader->value("param[@name='beta_w']/@value",&betaW);
  reader->value("param[@name='alpha_w']/@value",&alphaW);
  reader->value("param[@name='beta_r']/@value",&betaR);
  reader->value("param[@name='alpha_r']/@value",&alphaR);

  (*reader)>>std::pair<std::string,matrix*>("param[@name='R']/@value",&R0);
  (*reader)>>std::pair<std::string,matrix*>("param[@name='V']/@value",&V0);
  (*reader)>>std::pair<std::string,matrix*>("param[@name='Q']/@value",&Q);
  (*reader)>>std::pair<std::string,matrix*>("param[@name='W']/@value",&W);

  //Set default values for measurement
  V=V0;
  R=R0;

  if (reader->try_expression("line/@T1"))
  {
    line.setLine(reader->value<vector>("line/@T1"),
                 reader->value<vector>("line/@T2"));
  }
  this->Ts = reader->value<double>("param[@name='sampling-time']/@value");

  this->initModel();

  reader->try_value("param[@name='initial-state']/@value",&xk_1);
  this->x = xk_1;
}

void LFModel::initModel()
{
  //Transition matrix
  A = eye(stateNum);
  A(w,w) = 1 + Ts * -betaW/alphaW;
  A(w,buoyancy) = Ts/alphaW;
  A(r,r) = 1 + Ts * -betaR/alphaR;
  A(dV,vertCurrent) = Ts;
  A(dH,horzCurrent) = Ts;
  A(z,w) = Ts;
  A(psi,r) = Ts;

  //Input matrix
  B = mzeros(stateNum,2);
  B(w,Z) = 1/alphaW;
  B(r,N) = 1/alphaR;

  //Default
  x = zeros(stateNum);
}
