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
#ifndef SOIDENTIFICATION_HPP_
#define SOIDENTIFICATION_HPP_
#include <labust/control/Relay.hpp>
#include <vector>

namespace labust
{
  namespace control
  {
    /**
     * The class implements the model parameter identification procedure by use of Self Oscillations (Limit-cycles).
     *
     * \todo Check documentation.
     */
    class SOIdentification
    {
    public:
      /**
       * The parameters enumerator.
       */
      enum {alpha=0, kx=1, kxx=2, delta=3, wn=4, numParams};
      /**
       * Simple constructor.
       *
       * \param C Output amplitude of the relay.
       * \param X Input hysteresis of the relay.
       * \param R Reference for oscillations.
       * \param E Acceptable convergence error.
       * \param identLen The count of oscillations to track.
       */
      SOIdentification(double C = 0, double X = 0,
      		double R = 0, double E=0.1, double identLen = 4);
      /**
       * Generic destructor.
       */
      ~SOIdentification();

      /**
       * The method sets the symmetrical relay parameters from outside.
       *
       * \param C Output amplitude of the relay.
       * \param X Input hysteresis of the relay.
       */
      inline void setRelay(double C, double X)
      {
      	this->relay.setAmplitude(C);
      	this->relay.setHysteresis(X);
      }

      /**
       * The method to check if the identification procedure has finished.
       */
      inline bool isFinished(){return this->finished;}
      /**
       * The method returns the identified parameters of the model.
       */
      inline const std::vector<double>& parameters(){return params;}

      /**
       * The method implements the identification step.
       *
       * \param desired Desired value of the controlled state.
       * \param state Current value of the controlled state.
       * \param dT The sampling time between step calls.
       *
       * \return The desired control of the identification procedure.
       */
      double step(double error /*double desired, double state*/, double dT);
      /**
       * The method resets the identification phase.
       */
      void reset();

      /**
       * Set reference around which to identify.
       */
      void Ref(double ref){this->ref = ref;};
      /**
       * Set reference around which to identify.
       */
      double Ref(){return this->ref;};
      /**
       * The switch indicator.
       */
      inline bool hasSwitched(){return this->relay.hasSwitched();};
      /**
       * Return the current relative oscillation error.
       */
      inline double avgError(){return minMaxError;};

    private:
      /**
       * The parameter calculation when one cycle finishes.
       */
      void calculateParameters();

      /**
       * The identification progress flags.
       */
      bool finished;
      /**
       * The embedded relay controller.
       */
      labust::control::Relay relay;
      /**
       * The parameter container.
       */
      std::vector<double> params;
      /**
       * Timebase of the identification.
       */
      double tSum;
      /**
       * Oscillations maximum and minimum trackers.
       */
      double maxOsc, minOsc;
      /**
       * Oscillation counters.
       */
      int counterHigh, counterLow;
      /**
       * Error tolerance variables.
       */
      double eMaxError,eMinError, minMaxError;
      /**
       * Reference for identification.
       */
      double ref;
      /**
       * Length of the identification.
       */
      int identLen;
      /**
       * Data vetors.
       */
      std::vector<double> xa_high, xa_low, e_min, e_max, t_min, t_max;
    };
  };
};
/* SOIDENTIFICATION_H_ */
#endif
