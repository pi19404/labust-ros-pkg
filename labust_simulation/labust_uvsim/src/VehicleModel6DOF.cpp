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
#include <labust/simulation/VehicleModel6DOF.hpp>
#include <labust/math/Rotation.hpp>
#include <labust/xml/XMLuBlas.hpp>
#include <labust/xml/XMLReader.hpp>
#include <labust/math/uBlasOperations.hpp>

#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/banded.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace labust::simulation;

VehicleModel6DOF::VehicleModel6DOF():
     m(1),
     g_acc(9.81),
     B(0),
     dT(0.1),
     Io(eye(3)),
     Mrb(eye(6)),
     Ma(eye(6)),
     Crb(zero_m(6)),
     Ca(zero_m(6)),
     Dlin(eye(6)),
     Dquad(eye(6)),
     rg(zero_v(3)),
     rb(zero_v(3)),
     nu(zero_v(6)),
     nu0(zero_v(6)),
     eta(zero_v(6)),
     eta0(zero_v(6)),
     g(zero_v(6)),
     isCoupled(false),
     current(zero_v(3)){};

VehicleModel6DOF::VehicleModel6DOF(const labust::xml::ReaderPtr reader, const std::string& modelID):
     m(1),
     g_acc(9.81),
     B(0),
     dT(0.1),
     Io(eye(3)),
     Mrb(eye(6)),
     Ma(eye(6)),
     Crb(zero_m(6)),
     Ca(zero_m(6)),
     Dlin(eye(6)),
     Dquad(eye(6)),
     rg(zero_v(3)),
     rb(zero_v(3)),
     nu(zero_v(6)),
     nu0(zero_v(6)),
     eta(zero_v(6)),
     eta0(zero_v(6)),
     g(zero_v(6)),
     isCoupled(false),
     current(zero_v(3))
{
  this->configure(reader,modelID);
};

void VehicleModel6DOF::configure(const labust::xml::ReaderPtr reader, const std::string& modelID)
try
{
  _xmlNode* org_node = reader->currentNode();
  reader->useNode(reader->value<_xmlNode*>("VehicleModel" + (modelID.empty()?"":("[@id='" + modelID + "']"))));

  //Populate the model elements
  reader->value("sampling-time",&dT);
  reader->value("mass",&m);
  reader->value("gravity",&g_acc);
  (*reader)>>std::pair<std::string,matrix3*>("inertia-matrix",&Io);
  reader->value("rg",&rg);
  reader->value("rb",&rb);

  reader->value("lift-force",&B);
  (*reader)>>std::pair<std::string,matrix*>("added-mass",&Ma);
  (*reader)>>std::pair<std::string,matrix*>("linear-damping",&Dlin);
  (*reader)>>std::pair<std::string,matrix*>("quadratic-damping",&Dquad);
  (*reader).value("eta0",&eta0);
  (*reader).value("nu0",&nu0);

  this->reset();

  this->calculate_mrb();

  //Optional values
  reader->try_value("coupled",&isCoupled);
  reader->try_value("current",&current);
  //Configure the noise generator
  noise.configure(reader);

  //return original node
  reader->useNode(org_node);
}
catch (labust::xml::XMLException& e)
{
  e.append(" Check the configuration file.");
  throw;
}

VehicleModel6DOF::~VehicleModel6DOF(){};

void VehicleModel6DOF::setInertiaParameters(double m, const matrix3& Io,const vector3& rg, double g)
{
  this->m = m;
  this->Io = Io;
  this->rg = rg;
  this->g_acc = g;
  calculate_mrb();
}

void VehicleModel6DOF::calculate_mrb()
{
  using namespace labust::math;
  subrange(Mrb, 0,3, 0,3) = m*eye(3);
  subrange(Mrb, 0,3, 3,6) = -m*skewSymm3(rg);
  subrange(Mrb, 3,6, 0,3) = -m*skewSymm3(rg);
  subrange(Mrb, 3,6, 3,6) = Io;
}

void VehicleModel6DOF::step(const vector& tau)
{
  //Assemble the linear and angluar velocity transformation matrices
  matrix3 J1(labust::math::rotation_matrix()(eta(phi),eta(theta),eta(psi))),invJ1;
  matrix3 J2(labust::math::ang_vel_trans()(eta(phi),eta(theta),eta(psi)));

  //From body to world coordinates
  //\todo Add kinematic disturbance
  subrange(eta, 0,3) += dT*(prod(J1,subrange(nu, 0,3)) + current);
  subrange(eta, 3,6) += dT*prod(J2,subrange(nu, 3,6));
  //eta += dT*noise.calculateW();
  this->etaN = this->eta + this->noise.calculateW();

  //Calculate restoring forces
  restoring_force(J1);

  matrix M = Ma + Mrb;

  if (isCoupled)
  {
    matrix invM;

    if (labust::math::gjinverse(M,invM))
    {
      //Assemble coriolis
      coriolis();

      //Calculate the absolute values of the speed
      boost::numeric::ublas::vector<double> nu_abs(6);
      std::transform(nu.begin(),nu.end(),nu_abs.begin(),static_cast<double (*)(double)>(fabs));

      matrix Dabs = boost::numeric::ublas::diagonal_matrix<double>(nu_abs.size(),nu_abs.data());
      matrix D = Dlin + prod(Dquad,Dabs);
      matrix CD = Ca + Crb + D;
      vector CDv = prod(CD,nu);

      nu += dT*prod(invM,tau - CDv - g);
    }
    else
    {
      std::cout<<"Mass matrix is singular."<<std::endl;
    }
  }
  else
  {
  	//Simplification for uncoupled
    for (size_t i=0; i<nu.size(); ++i)
    {
      double beta = Dlin(i,i) + Dquad(i,i)*fabs(nu(i));
      nu(i) += dT/(M(i,i))*(tau(i) - beta*nu(i));
    };
  }
  this->nuN = this->nu + this->noise.calculateV();
  //nu += noise.calculateV();
}

void VehicleModel6DOF::coriolis()
{
  using namespace labust::math;
  //We assume that Mrb and Ma matrices are set
  vector3 nu1 = subrange(nu, 0, 3);
  vector3 nu2 = subrange(nu, 3, 6);

  //subrange(Crb, 0,3, 0,3) = zero_m(3);
  subrange(Crb, 0,3, 3,6) = -m*skewSymm3(nu1) - m*prod(skewSymm3(nu2),skewSymm3(rg));
  subrange(Crb, 3,6, 0,3) = -m*skewSymm3(nu1) + m*prod(skewSymm3(rg),skewSymm3(nu2));
  subrange(Crb, 3,6, 3,6) = -skewSymm3(prod(Io,nu2));

  matrix3 M11=subrange(Ma, 0,3, 0,3);
  matrix3 M12=subrange(Ma, 0,3, 3,6);
  matrix3 M21=subrange(Ma, 3,6, 0,3);
  matrix3 M22=subrange(Ma, 3,6, 3,6);

  //subrange(Ca, 0,3, 0,3) = zero_m(3);
  subrange(Ca, 0,3, 3,6) = -skewSymm3(prod(M11,nu1) + prod(M12,nu2));
  subrange(Ca, 3,6, 0,3) = -skewSymm3(prod(M11,nu1) + prod(M21,nu2));
  subrange(Ca, 3,6, 3,6) = -skewSymm3(prod(M21,nu1) + prod(M22,nu2));
}

void VehicleModel6DOF::restoring_force(const matrix3& J1)
{
  using namespace boost::numeric::ublas;
  using namespace labust::math;

  unit_vector<double> id(3,2);

  matrix3 invJ1;
  inverse(J1,invJ1);

  vector3 fg = prod(invJ1,m*g_acc*id);
  vector3 fb = -prod(invJ1,B*id);

  subrange(g, 0,3) = -fg-fb;
  subrange(g, 3,6) = -prod(skewSymm3(rg),fg)-prod(skewSymm3(rb),fb);
}

std::ostream& labust::simulation::operator<<(std::ostream& s,const VehicleModel6DOF& model)
{
	using boost::numeric::ublas::operator<<;
  s<<"Sample time:"<<model.dT<<std::endl;
  s<<"Current state:\n\t Nu:"<<model.nu<<"\n\tEta:"<<model.eta<<std::endl;
  s<<"Inertia matrices:\n\tMrb:"<<model.Mrb<<"\n\t Ma:"<<model.Ma<<std::endl;
  s<<"Coriolis matrices:\n\tCrb:"<<model.Crb<<"\n\t Ca:"<<model.Ca<<std::endl;
  s<<"Damping matrices:\n\t Dlin:"<<model.Dlin<<"\n\tDquad:"<<model.Dquad<<std::endl;
  s<<"Gravity and buoyancy:\n\trg:"<<model.rg<<"\n\trb:"<<model.rb<<std::endl;
  s<<"Inertia tensor:\n\tIo:"<<model.Io<<std::endl;
  s<<"Restoring forces:\n\tg:"<<model.g<<std::endl;
  s<<"Other:\n\tg_acc:"<<model.g_acc<<"\n\t    B:"<<model.B<<"\n\t    m:"<<model.m;
  if (model.isCoupled)
    s<<"\nThe model uses coupled dynamics."<<std::endl;
  else
    s<<"\nThe model uses uncoupled dynamics."<<std::endl;
  return s;
}
