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
#ifndef PIDCONTROLLER_HPP_
#define PIDCONTROLLER_HPP_
#include <labust/control/Proportional.hpp>
#include <labust/control/Integrator.hpp>
#include <labust/control/Derivative.hpp>

#include <labust/math/Limits.hpp>
#include <labust/math/NumberManipulation.hpp>

#include <boost/mpl/identity.hpp>

#include <iostream>

namespace labust
{
	namespace control
	{
		namespace details
		{
			/**
			 * The class defines a variable controller structure.
			 */
			class VariableStructure
			{
			public:
				/**
				 * Generic constructor.
				 */
				VariableStructure():alpha(1),beta(1),gamma(1){};
				/**
				 * Set the structure parameters.
				 */
				inline void setStructure(double alpha, double beta, double gamma)
				{
					this->alpha=alpha;
					this->beta=beta;
					this->gamma=gamma;
				}

			protected:
				/**
				 * Setpoint gains
				 */
				double alpha, beta, gamma;
			};
			/**
			 * Fixed structure IPD controller.
			 */
			class IPD
			{
			protected:
				enum {alpha = 0, gamma = 0, beta = 1};
			};
			/**
			 * Fixed structure PID controller.
			 */
			class PID
			{
			protected:
				enum {alpha = 1, gamma = 1, beta = 1};
			};
		}

		/**
		 * This class implements a scalar PID controller. Different types can be achieved
		 * by combination of control parameters alpha, beta, gamma.
		 *
		 * \todo Add compensation for PID parameter change. From Astrom's book. Needed for
		 * the line following controller and similar real-time tuning controllers.
		 *
		 * \todo Add the structure parameter in the constructor and force initial structure definition.
		 * \todo Structure can be a template parameter with 3 variables.
		 */
		template<
		class Structure = labust::control::details::VariableStructure,
		class P_LP = NoLimits,
		class I_LP = NoLimits,
		class D_LP = NoLimits>
		class PIDController : public Structure
		{
		public:
			typedef Structure Structure;
			/**
			 * Main constructor. Users can specify default gains of the controller.
			 *
			 * \param Kp Gain of the proportional controller.
			 * \param Ki Gain of the integral controller.
			 * \param Kd Gain of the derivative controller.
			 * \param Tf Time constant of the derivative filter.
			 * \param Ts Sampling time
			 */
			PIDController(double Kp, double Ki, double Kd, double Tf = 0, double Ts = 0.1):
				P(Kp),
				I(Ki,Ts),
				D(Kd,Tf,Ts),
				windup(false){};
			/**
			 * Generic constructor.
			 *
			 * \param structure The structure of the controller.
			 */
			PIDController():
				windup(false){};

			/**
			 * Reinitializes the controller.
			 */
			inline void reset(){this->I.reset(); this->D.reset();};
			/**
			 * This is the main method that calculates the output based on the
			 * desired and current value.
			 * \param desired Desired value of the controlled state
			 * \param state Current value of the controlled state
			 * \param fl The feedback linearization argument.
			 */
			inline double step(double desired, double state, double fl=0)
			{
				double u = this->P.step(this->alpha*desired - state)
      					+ this->D.step(this->gamma*desired - state)
      					+ this->I.step()
      					+ fl;

				double u_l = labust::math::coerce(u,this->outlim);

				if (!(this->windup = ((u_l != u) && (u*(desired-state)>0))))
				{
					I.step(this->beta*desired - state);
				}

				return u_l;
			};
			/**
			 * This is the main method that calculates the output based on the
			 * desired and current value. Error between the desired and current
			 * state is supplied separately because some error need wrapping.
			 * By default it has a behaviour of I_PD without limits.
			 *
			 * \param desired Desired value of the controlled state
			 * \param state Current value of the controlled state
			 * \param error Error difference between the desired and current state
			 */
			//inline double step(double desired, double state, double error){};

			/**
			 * Sets the controller gains.
			 *
			 * \param Kp Gain of the proportional controller.
			 * \param Ki Gain of the integral controller.
			 * \param Kd Gain of the derivative controller.
			 * \param Tf Time constant of the derivative filter.
			 */
			inline void setGains(double Kp, double Ki, double Kd, double Tf)
			{
				this->P.setKp(Kp);
				this->I.setKi(Ki);
				this->D.setKd(Kd);
				this->D.setTf(Tf);
			}
			/**
			 * Sets the internal state of the controller.
			 *
			 * \param intState Integrator internal state.
			 * \param derState Derivative internal state.
			 */
			inline void initialize(double intState, double derState)
			{
				this->I.setState(intState);this->D.setState(derState);
			}
			/**
			 * Sets the sampling time of the controllers.
			 *
			 * \param Ts Sampling time.
			 */
			inline void setTs(double Ts)
			{
				this->I.setTs(Ts);this->D.setTs(Ts);
			}
			/**
			 * This method sets the input output limits. Output limits are used for anti-windup.
			 * Input limits act on the desired (reference) value.
			 *
			 * \param limits The desired input-output limits
			 */
			inline void setLimits(const labust::math::Limit<double>& limits){this->outlim = limits;};
			/**
			 * This method sets the input output limits for the P part.
			 *
			 * \param limits The desired output limits.
			 */
			inline void setPLimits(const labust::math::Limit<double>& limits){this->P.setLimits(limits);};
			/**
			 * This method sets the input output limits for the I part.
			 *
			 * \param limits The desired output limits.
			 */
			inline void setILimits(const labust::math::Limit<double>& limits){this->P.setLimits(limits);};
			/**
			 * This method sets the input output limits for the D part.
			 *
			 * \param limits The desired output limits.
			 */
			inline void setDLimits(const labust::math::Limit<double>& limits){this->P.setLimits(limits);};

		private:
			/**
			 * Proportional action of the controller.
			 */
			Proportional<P_LP> P;
			/**
			 * The integral action of the controller.
			 */
			Integrator<I_LP> I;
			/**
			 * Deriative action of the controller.
			 */
			Derivative<D_LP> D;
			/**
			 * Output limit
			 */
			labust::math::Limit<double> outlim;
			/**
			 * Windup flag.
			 */
			bool windup;
		};

		/**
		 * Generic PID controller type defintion.
		 */
		typedef PIDController<labust::control::details::PID> PID;
		/**
		 * Generic IPD controller type defintion.
		 */
		typedef PIDController<labust::control::details::IPD> IPD;
		/**
		 * Generic LF controller PD type defintion.
		 */
		typedef PIDController<labust::control::details::PID,UseLimits> lfPD;
	};
};
/* PIDCONTROLLER_HPP_ */
#endif
