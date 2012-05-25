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
#ifndef CONTROLFWD_HPP_
#define CONTROLFWD_HPP_

namespace labust
{
	namespace control
	{
		/**
		 * General controller tuning parameters. Useful for vehicle controller tuning.
		 */
		struct TuningParameters
		{
			/**
			 * Generic uncoupled model parameters.
			 */
			double alpha,beta,betaa;
			/**
			 * Binomial model function frequency.
			 */
			double w;
			/**
			 * Symmetric output saturation.
			 */
			double max;
		};

		/**
		 * Generic controller tuning.
		 *
		 * \param param Tuning parameters for the controller.
		 * \param pid Address of the PID controller to tune.
		 *
		 * \tparam PID Allow the function for different controllers.
		 */
		void tuneController(const TuningParameters& param, IPD* pid)
		{
		  double a3 = 1/(param.w*param.w*param.w), a2 = 3/(param.w*param.w), a1 = 3/(param.w);

		  double K_Ipsi = param.alpha/a3;
		  double K_Ppsi = param.alpha*a1/a3;
		  double K_Dpsi = (param.alpha*a2/a3 - param.beta);

		  //pid->setGains(a1*param.alpha/a3,param.alpha*a3,a2/a3*param.alpha - param.beta,0);
		  pid->setGains(K_Ppsi,K_Ipsi,K_Dpsi,K_Dpsi/10);

		  labust::math::Limit<double> limit(-param.max,param.max);
		  pid->setLimits(limit);
		}
	}
}

/* CONTROLFWD_HPP_ */
#endif
