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
#ifndef RELAY_HPP_
#define RELAY_HPP_

namespace labust
{
  namespace control
  {
    /**
     * The class implements a simple relay switch. This can be used for bang-bang control.
     *
     * \todo Check documentation.
     */
    class Relay
    {
    public:
      /**
       * Generic constructor.
       */
      Relay():
      	Cp(1),Cm(-1),Xap(0),Xam(0),switched(false),out(1){};
      /**
       * Main constructor. Specifies the relay parameters.
       *
       * \param C Output amplitude -C, C.
       * \param X Input hysteresis -Xa, Xa.
       */
      Relay(double C, double X):
      	Cp(C),Cm(-C),Xap(X),Xam(-X),switched(false),out(C){};
      /**
       * Expanded constructor that allows for non-symmetrical relay specification.
       *
       * \param Cp Positive output amplitude.
       * \param Cm Negative output amplitude.
       * \param Xp Positive input hysteresis.
       * \param Xm Negative input hysteresis.
       */
      Relay(double Cp, double Cm, double Xp, double Xm):
      	Cp(Cp),Cm(Cm),Xap(Xp),Xam(Xm),switched(false),out(Cp){};
      /**
       * Generic destructor.
       */
     ~Relay(){};

      /**
       * The method sets the relay output amplitude.
       *
       * \param C Output amplitude -C, C.
       */
      inline void setAmplitude(double C){this->out = this->Cp = C; this->Cm = -C;};
      /**
       * \override setAmplutde
       * Convenience override for the non-symmetrical case.
       *
       * \param Cp Positive output amplitude.
       * \param Cm Negative output amplitude.
       */
      inline void setAmplitude(double Cp, double Cm){this->out = this->Cp = Cp; this->Cm = Cm;};
      /**
       * The method sets the relay input hysteresis.
       *
       * \param X Input hysteresis value.
       */
      inline void setHysteresis(double X){this->Xam = -X; this->Xap = X;};
      /**
       * \override setHysteresis
       * Conveniece override for the non-symmetrical case.
       *
       * \param Xap Input hysteresis positive value.
       * \param Xam Input hysteresis negative value.
       */
      inline void setHysteresis(double Xp, double Xm){this->Xam = Xm; this->Xap = Xp;};
      /**
       * The method returns the absolute value of the amplitude.
       *
       * \return The amplitude of the relay.
       */
      inline double getAmplitude(){return this->Cp;};

      /**
       * The method returns if the relay has switched since the last step call.
       *
       * \return 1 if the switch to high happened and -1 if the switch to low happened. Zero otherwise.
       */
      inline int hasSwitched()
      {
      	if (this->switched)
      	{
      		if (this->out == this->Cp) return 1;
      		else return -1;
      	}
      	return 0;
      }
      /**
       * The method performs one step. It compares the input value to the hysteresis values
       * and determines weather to switch.
       *
       * \param input Input to the relay.
       * \return The relay output.
       */
      double step(double input)
      {
      	if ((this->switched = ((input>=Xap) && (this->out != this->Cp)))) this->out = this->Cp;
      	else if ((this->switched = ((input<=this->Xam) && (this->out != this->Cm)))) this->out = this->Cm;

      	return this->out;
      }

    private:
      /**
       * Relay amplitudes.
       */
      double Cp, Cm;
      /**
       * Relay hysteresis.
       */
      double Xap,Xam;
      /**
       * Switch indicator.
       */
      bool switched;
      /**
       * Relay output for switch detection.
       */
      double out;
    };
  };
};
/* RELAY_HPP_ */
#endif
