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
#include <labust/xml/xmlfwd.hpp>

#include <boost/numeric/ublas/matrix.hpp>


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
      enum {Xa_high = 0, Xa_low, E_min, E_max, T_min, T_max};
      enum {numParams = 4};
      enum {identLen=6};
      enum {identParamNum=6};
    public:
      /**
       * The parameters enumerator.
       */
      enum {alpha=0, kx=1, kxx=2, delta=3};
      /**
       * Parameters container typedef.
       */
      typedef boost::array<double,numParams> ParameterContainer;
      /**
       * The data matrix typedef.
       */
      typedef boost::numeric::ublas::matrix<double> DataMatrix;
      /**
       * Generic constructor.
       */
      SOIdentification();
      /**
       * Main constructor. Configures the identification step based on the
       * XML configuration.
       *
       * \param reader Pointer to the XML configuration reader.
       */
      SOIdentification(const labust::xml::ReaderPtr reader);
      /**
       * Helper constructor. Allows for direct specification of identification parameters.
       *
       * \param reader Pointer to the XML configuration reader.
       */
      //SOIdentification(...)
      /**
       * Generic destructor.
       */
      ~SOIdentification();

      /**
       * The method configures the object with the help of the XML configuration.
       *
       * \param reader Pointer to the XML configuration reader.
       */
      void configure(const labust::xml::ReaderPtr reader);

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
       *
       * \param params The identified parameters.
       *
       */
      bool parameters(ParameterContainer* const params)
      {
      	(*params) = this->params;
      	return this->successful;
      }

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

    private:
      /**
       * The parameter calculation when one cycle finishes.
       */
      void calculateParameters();

      /**
       * The identification progress flags.
       */
      bool finished, successful;
      /**
       * The embedded relay controller.
       */
      labust::control::Relay relay;
      /**
       * The parameter container.
       */
      ParameterContainer params;

      /**
       * Data matrix. Contains all information collected during identification.
       */
      DataMatrix buffer;
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
      size_t counterHigh, counterLow;
      /**
       * Error tolerance variables.
       */
      double eMaxError,eMinError;
    };
  };
};

#endif /* SOIDENTIFICATION_H_ */
