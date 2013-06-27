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
*  Author: Dula Nad
*  Created: 01.02.2010.
*********************************************************************/
#ifndef RBMODEL_HPP_
#define RBMODEL_HPP_
#include <labust/simulation/matrixfwd.hpp>
#include <labust/simulation/NoiseModel.hpp>
#include <labust/math/NumberManipulation.hpp>

namespace labust
{
  namespace simulation
  {
    /**
     *  This class implements a 6DOF marine vehicle model. This model is based on
     *  the equations of motion derived in chapter 2 of Guidance and control of
     *  Ocean Vehicles by Fossen (1994).
     */
    class RBModel
    {
    public:
      enum {x=0,y,z,phi,theta,psi};
      enum {u=0,v,w,p,q,r};
      enum {X=0,Y,Z,K,M,N};
      /**
       * Default constructor for common use without configuration.
       */
      RBModel();
      /**
       * Generic destructor.
       */
      ~RBModel();

      /**
       * The method performs one simulation step. The model is propagated in time for one sampling period.
       * To access states after the propagation use the accessor methods.
       *
       * \param tau Vector of forces and moments acting on the model.
       */
      void step(const vector& tau);

      /**
       * Method to get the current model states. This vector returns the model position and orientation
       * in world coordinates.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& Eta() const {return this->eta;};
      /**
       * Method to get the current model states. This vector returns the model linear and rotational speeds in
       * the body-fixed coordinate frame.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& Nu() const {return this->nu;};
      /**
       * Method to get the current model acceleration. This vector returns the model linear and rotational accelerations in
       * the body-fixed coordinate frame.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& NuAcc() const {return this->nuacc;};
      /**
       * Method to get the current model states. This vector returns the model position and orientation
       * in world coordinates.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& EtaNoisy() const {return this->etaN;};
      /**
       * Method to get the current model states. This vector returns the model linear and rotational speeds in
       * the body-fixed coordinate frame.
       *
       * \return Constant reference to the model states.
       */
      inline const vector& NuNoisy() const {return this->nuN;};

      /**
       * The method sets the sampling time of the model simulation.
       *
       * \param dT The desired sampling time in seconds.
       */
      inline void setTs(double dT){this->dT = dT;};
      /**
       * The method sets the mass, inertia tensor and CoG (Center of Gravity) vector. These values are use to calculate the rigid-body
       * mass-inertia matrix (Mrb). The
       *
       * \param m Model mass.
       * \param Io Rigid-body inertia matrix.
       * \param rg Center of gravity vector.
       * \param g Gravity acceleration (defaults to 9.81).
       */
      void setInertiaParameters(double m, const matrix3& Io,const vector3& rg, double g = 9.81);
      /**
       * The method sets the added mass matrix of the model.
       *
       * \param Ma The model added mass matrix.
       */
      inline void setAddedMass(const matrix& Ma){this->Ma = Ma;};
      /**
       * The method sets the linear and quadratic damping matrices of the model.
       *
       * \param Dlin Linear part of the damping matrix.
       * \param Dquad Quadratic part of the damping matrix.
       */
      inline void setDampingMatrix(const matrix& Dlin,const matrix& Dquad)
      {
        this->Dlin = Dlin;
        this->Dquad = Dquad;
      }
      /**
       * The method sets the buoyancy force and CoB (Center of Buoyancy) of the dynamic model.
       *
       * \param B Lift force
       * \param rb Center of buoyancy
       */
      inline void setBuoyancyInfo(double rho, const vector3& rb)
      {
        this->rb = rb;
      }
      /**
       * The method sets the bounding ellipsoid of the vehicle.
       *
       * \param ae length of x axis
       * \param be length of y axis
       * \param ce length of z axis
       */
      inline void setBoundingEllipsoid(double ae, double be, double ce)
      {
        this->ae = ae;
        this->be = be;
        this->ce = ce;
      }
      /**
       * The method sets the water level.
       *
       * \param B Lift force
       * \param rb Center of buoyancy
       */
      inline void setWaterLevel(double waterLevel)
      {
        this->waterLevel = waterLevel;
      }
      /**
       * The method sets the initial states of the model.
       *
       * \param nu Linear and rotational velocities in the body-fixed coordinate frame.
       * \param eta Position and orientation in the earth-fixed coordinate frame.
       */
      inline void setNuAndEta(const vector& nu,const vector& eta)
      {
      	this->nuN = this->nu = this->nu0 = nu;
      	this->etaN = this->eta = this->eta0 = eta;
      }
      /**
       * Selects the model simulation method. In coupled operations the model DOF are interconnected as in real-life scenarios.
       * Uncoupled operation simulates each model's DOF separately. Useful when parameters for some DOF are unknown.
       *
       * \param coupled True if the model should be coupled.
       */
      inline void coupled(bool coupled){this->isCoupled = coupled;};
      /**
       * Set the external current disturbance.
       *
       * \param current The 3D vector of the current under effect.
       */
      inline void setCurrent(const vector3& current){this->current = current;}
      /**
       * Returns the pressure based on depth.
       */
      inline double getPressure(double h){return this->rho*this->g_acc*h;};

      /**
       * The method restarts the model to initial parameters.
       */
      inline void reset()
      {
        this->etaN = this->eta = this->eta0;
        this->nuN = this->nu = this->nu0;
        this->B=2*labust::math::coerce((eta(z)+waterLevel)/ce+1,0,2)*M_PI/3*ae*be*ce*rho*g_acc;
      };

    protected:
      /**
       * The method calculates the Coriolis matrix of the model.
       */
      void coriolis();
      /**
       * The method calculates the restoring forces.
       */
      void restoring_force(const matrix3& J1);
      /**
       * The method calculates the rigid-body mass matrix of the model.
       */
      void calculate_mrb();

      /**
       * Model mass, water density, gravity, bounding ellipsoid sides,
       * sampling time, water level, buoyancy.
       */
      double m,g_acc,rho,ae,be,ce,dT,waterLevel,B;
      /**
       * Inertia matrix of the model.
       */
      matrix3 Io;
      /**
       * Rigid-body mass and added mass matrices.
       */
      matrix Mrb,Ma;
      /**
       * The coriolis and centripetal matrices of the model and the added mass.
       */
      matrix Crb,Ca;
      /**
       * Linear and quadratic parts of the damping matrix.
       */
      matrix Dlin,Dquad;
      /**
       * Center of gravity and buoyancy.
       */
      vector3 rg,rb;
      /**
       * Linear and orientation velocity and their initial state.
       */
      vector nu,nu0,nuN,nuacc;
      /**
       * Position and orientation and their initial state.
       */
      vector eta,eta0,etaN;
      /**
       * The restoring forces vector.
       */
      vector g;
      /**
       * Coupled dynamics flag and depth limit.
       */
      bool isCoupled, positiveDepth;
      /**
       * The noise generator.
       */
      labust::simulation::NoiseModel noise;
      /**
       * The external current disturbance vector.
       */
      vector3 current;
    };

    /**
     * The function computes the skew-symmetric matrix from a given vector.
     * The function operates with vector size 3.
     *
     * \param vec The desired vector of size 3.
     *
     * \tparam in The input matrix type.
     *
     * \return The calculated 3x3 skew symmetric matrix.
     */
    template<class Derived>
    matrix3 skewSymm3(const Eigen::MatrixBase<Derived>& vec)
    {
      matrix3 mat;
      mat<<0,-vec(2),vec(1),
      		vec(2),0,-vec(0),
      		-vec(1),vec(0),0;
      return mat;
    };
  }
}

/* RBMODEL_HPP_ */
#endif
