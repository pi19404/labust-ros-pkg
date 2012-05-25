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
#ifndef INTEGRATOR_HPP_
#define INTEGRATOR_HPP_

#include <labust/control/LimitPolicies.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <stdexcept>

namespace labust
{
  namespace control
  {
    /**
     * This class implements a scalar integrator.
     * You can specify the following policies I/O limits policy.
     *
     * Following policy extensions would be usefull:
     *  1. Precission selector (float, double)
     *  2. Discretization selector (maybe)
     */
    template <class LimitPolicy = NoLimits>
    class Integrator : public LimitPolicy
    {
    public:
      /**
       * Main constructor for the integrator class.
       *
       * \param Ki Gain of the integration
       * \param state Initial state of the integrator.
       * \param Ts Sampling time.
       */
      Integrator(double Ki = 1, double state = 0, double Ts = 0.1):
        state(0),
        Ki(Ki),
        Ts(Ts)
      {
        //Cannot except zero sampling time.
        if (!Ts) throw std::invalid_argument("Can not define zero sampling time.");
      };

      /**
       * This method sets the internal state of the integrator.
       *
       * \param The desired internal state
       */
      inline void setState(double state){this->state = state;};
      /**
       * This method sets the integrator gain.
       *
       * \param Ki Integrator gain
       */
      inline void setKi(double Ki){this->Ki = Ki;};
      /**
       * This method sets the sampling time.
       *
       * \param Ts New desired sampling time
       */
      inline void setTs(double Ts)
      {
        Ts ? (this->Ts = Ts) : (throw std::invalid_argument("Can not define zero sampling time."));
      };

      /**
       * This method performs one integration step. Notice that step()
       * will return the current state value, without making any changes
       * to the state.
       *
       * \param input Input into the integrator
       * \param dT Time difference on which to perform integration
       * \return Propagated state of the integrator.
       */
      inline double step(double input = 0)
      {
        return state += Ki*input*Ts;
      }
      /**
       * This methods set the internal state to zero.
       */
      inline void reset(void){this->state = 0;};

    private:
      /**
       * Internal state
       */
      double state;
      /**
       * Integration gain
       */
      double Ki;
      /**
       * Sampling time
       */
      double Ts;
    };

    /**
     * Specialization when using limits.
     */
    template <>
    inline double Integrator<UseLimits>::step(double input)
    {
      return state=labust::math::coerce(state += Ki*input*Ts,limits);
    };
  }
}
/* INTEGRATOR_HPP_ */
#endif
