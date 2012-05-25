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
#ifndef KFBASE_HPP_
#define KFBASE_HPP_
#include <labust/math/uBlasOperations.hpp>

//#include <boost/call_traits.hpp>

namespace labust
{
	namespace navigation
	{
		/**
		 * The base class for the Kalman filter.
		 */
		template <class Model>
		class KFBase : public Model
		{
		public:
			typedef const typename Model::matrix& matrixref;
			typedef const typename Model::vector& vectorref;
			//For MIMO systems
			typedef const typename Model::input_type& inputref;
			typedef const typename Model::output_type& outputref;

			/**
			 * Generic constructor.
			 */
			KFBase(): P(1000*typename Model::eye(Model::stateNum)){};

			/**
			 * Set the state covariance matrix value.
			 *
			 * \param P State covariance matrix.
			 */
			inline void setStateCovariance(matrixref P){this->P = P;};
			/**
			 * Get the state covariance matrix value.
			 *
			 * \return State covariance matrix.
			 */
			inline matrixref getStateCovariance(){return this->P;};
			/**
			 * Set the estimated state value.
			 *
			 * \param x The state value.
			 */
			inline void setState(vectorref x){this->xk_1 = x; this->x = x;};
			/**
			 * Set the state value.
			 *
			 * \return The estimated state value.
			 */
			inline vectorref getState(){return this->x;};
			/**
			 * Set the desired sampling time.
			 *
			 * \param Ts Sampling time
			 */
			inline void setTs(double Ts)
			{
				if (!Ts) throw std::invalid_argument("Cannot set zero sampling time.");
				this->Ts = Ts;
				this->initModel();
			};

			/**
			 * Get the covariance trace.
			 *
			 * \return Trace of the covariance matrix.
			 */
			inline double traceP()
			{
				return labust::math::trace(P);
			}

		protected:
			/**
			 * The Kalman gain, estimate covariance and the innovation covariance matrix
			 */
			typename Model::matrix K, P, innovationCov;
			/**
			 * The innovation
			 */
			typename Model::vector innovation;
		};
	}
}

/* KFBASE_HPP_ */
#endif
