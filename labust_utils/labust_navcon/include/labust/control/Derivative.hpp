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
#ifndef DERIVATIVE_HPP_
#define DERIVATIVE_HPP_

#include <labust/control/LimitPolicies.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <stdexcept>

namespace labust
{
  namespace control
  {
    /**
     * This class implements the real derivative in a discrete form. It contains the usual
     * first order filter to make the derivator causal.
     * You can choose the limits policy.
     *
     * Following policy extensions would be usefull:
     *  1. Precission selector (float, double)
     *  2. Discretization selector (maybe)
     */
    template <class LimitPolicy = NoLimits>
    class Derivative : protected LimitPolicy
    {
	public:
      /**
       * Generic constructor. Define a filtering time if needed.
       *
       * \param Kd Gain of the derivative action.
       * \param Tf Time constant for the first order filter.
       * \param Ts Sampling time
       */
      Derivative(double Kd = 1, double Tf = 0, double Ts = 0.1):
        output(0),
        lastInput(0),
        Tf(Tf),
        Kd(Kd),
        Ts(Ts)
      {
        //Cannot except zero sampling time.
        if (!Ts) throw std::invalid_argument("Can not define zero sampling time.");
      };

      /**
       * This method sets the internal state of the derivator.
       *
       * \param The desired internal state.
       */
      inline void setState(double state){this->lastInput = state;};
      /**
       * This method set the desired time constant of the filter.
       *
       * \param Tf Time constant for the real derivative
       */
      inline void setTf(double Tf){this->Tf = Tf;};
      /**
       * This method set the desired derivative gain.
       *
       * \param Kd Derivative gain
       */
      inline void setKd(double Kd){this->Kd = Kd;};
      /**
       * This method ster the sampling time.
       *
       * \param Ts New desired sampling time
       */
      inline void setTs(double Ts)
      {
        Ts ? (this->Ts = Ts) : (throw std::invalid_argument("Can not define zero sampling time."));
      };

      /**
       * This method calculates the next output of the derivative.
       *
       * \param input Input into the derivative
       * \param dT Sampling period to be used during calculation
       * \return New calculated state.
       */
      double step(double input)
      {
        output = (Tf*output + Kd*(input - lastInput))/(Ts + Tf);
        lastInput = input;
        return output;
      }

      /**
       * Resets the derivative to its initial state.
       */
      inline void reset(void){output = 0; lastInput = 0;};

    private:
      /**
       * Derivative output used in the following derivation step.
       */
      double output;
      /**
       * Last input to the derivative.
       */
      double lastInput;
      /**
       * Time constant of the filter.
       */
      double Tf;
      /**
       * Gain of the derivative action.
       */
      double Kd;
      /**
       * Sampling time.
       */
      double Ts;
    };

    //Specialization when using limits.
    template<>
    inline double Derivative<UseLimits>::step(double input)
    {
    	//Calculate new state
    	output = labust::math::coerce((Tf*output + Kd*(input -  lastInput))/(Ts + Tf),limits);
    	lastInput = input;

    	return output;
    }
  }
}

/* DERIVATIVE_HPP_ */
#endif
