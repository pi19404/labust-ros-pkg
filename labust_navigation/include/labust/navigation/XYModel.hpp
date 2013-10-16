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
#ifndef XYMODEL_HPP_
#define XYMODEL_HPP_
#include <labust/navigation/SSModel.hpp>

namespace labust
{
  namespace navigation
  {
    /**
     * This class implements a x-y surface dynamic model for the EKF filter.
     */
    class XYModel : public SSModel<double>
    {
    	typedef SSModel<double> Base;
    public:
      typedef vector input_type;
      typedef vector output_type;

      struct ModelParams
      {
    	  ModelParams():
    		  alpha(1),
    		  beta(1),
    		  betaa(0){};

    	  ModelParams(double alpha, double beta, double betaa):
    		  alpha(alpha),
    		  beta(beta),
    		  betaa(betaa){}

    	  inline double Beta(double val)
    	  {
    		  return beta + betaa*fabs(val);
    	  }

    	  double alpha, beta, betaa;
      };

      enum {u=0,v,r,xp,yp,psi,xc,yc,b1,b2};
      enum {stateNum = 10};
      enum {inputSize = 3};
      enum {measSize = 3};
      enum {X=0,Y,N};
      enum {x_m=0,y_m,psi_m,psiOnly_m = 0};
        
      /**
       * The default constructor.
       */
      XYModel();
      /**
       * Generic destructor.
       */
      ~XYModel();

      /**
       * Perform a prediction step based on the system input.
       *
       * \param u System input.
       */
      void step(const input_type& input);
      /**
       * Calculates the estimated output of the model.
       *
       * \param y Inserts the estimated output values here.
       */
      void estimate_y(output_type& y);
      /**
       * Initialize the model to default values
       */
      void initModel();

      /**
       * Setup the measurement matrix for available measurements.
       */
      const output_type& update(vector& measurements, vector& newMeas);

      /**
       * Setup the measurement matrix for heading only.
       */
      const output_type& yawUpdate(double yaw);
      /**
       * Setup the measurement matrix for full update.
       */
      const output_type& fullUpdate(double x,
    		  double y,
    		  double yaw);
      /**
       * Setup the measurement matrix for position update.
       */
      const output_type& positionUpdate(double x,
    		  double y);
      /**
       * Set the model parameters.
       */
      void setParameters(const ModelParams& surge,
    		  const ModelParams& sway,
    		  const ModelParams& yaw)
      {
    	  this->surge = surge;
    	  this->sway = sway;
    	  this->yaw = yaw;
      }

      void calculateXYInovationVariance(const matrix& P, double& xin,double &yin);

      /**
       * Return the speeds in the local frame.
       */
      inline void getNEDSpeed(double& xdot, double& ydot)
      {
      	xdot = this->xdot;
      	ydot = this->ydot;
      }


    protected:
     /**
       * Calculate the Jacobian matrices.
       */
      void derivativeAW();
      /**
        * Calculate the Jacobian matrices.
        */
       void derivativeHV(int num);
      /**
       * The model parameters.
       */
      ModelParams surge,sway,yaw;
      /**
       * The newest measurement.
       */
      output_type measurement;
      /**
       * The NED speeds.
       */
      double xdot,ydot;
    };
  }
}

/* XYMODEL_HPP_ */
#endif
