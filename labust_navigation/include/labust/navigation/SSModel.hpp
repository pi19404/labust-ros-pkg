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
#ifndef SSMODEL_HPP_
#define SSMODEL_HPP_
#include <Eigen/Dense>
#include <boost/noncopyable.hpp>

namespace labust
{
	namespace navigation
	{
		/**
		 * This class implements a generic stochastic state space model. When implementing models
		 * try to inherit from this class. The following naming scheme is used:
		 *
		 * x(k+1) = A*x(k) + B*u(k) + w(k)
		 * y(k) = H*x(k) + v(k)
		 *
		 * w - model noise p(w) = N(0,Q(k))
		 * v - measurement noise p(v) = N(0,R(k))
		 *
		 * In the nonlinear case:
		 *
		 * x(k+1) = f(x(k),u(k),w(k))
		 * y(k) = h(x(k),v(k))
		 *
		 * update the following jacobian matrices as:
		 * A = \frac{\partial{f}}{\partial{x}}(x(k-1),u(k),0)
		 * W = \frac{\partial{f}}{\partial{w}}(x(k-1),u(k),0)
		 * H = \frac{\partial{h}}{\partial{x}}(x(k),0)
		 * V = \frac{\partial{h}}{\partial{v}}(x(k),0)
		 *
		 * We leave the matrix size generic for now. Usually these could be supplied directly in the
		 * template.
		 *
		 * \todo Deprecate R0/R as there can be only one of them and by selecting V/V0 appropriately we
		 * can achieve the same result with one matrix less.
		 */
		template <class precission = double>
		class SSModel : boost::noncopyable
		{
		public:
			typedef precission numericprecission;

			typedef Eigen::Matrix<precission, Eigen::Dynamic, Eigen::Dynamic> matrix;
			typedef Eigen::Matrix<precission, Eigen::Dynamic, 1> vector;

			//typedef boost::numeric::ublas::identity_matrix<precission> eye;
			//typedef boost::numeric::ublas::zero_vector<precission> zeros;
			//typedef boost::numeric::ublas::zero_matrix<precission> mzeros;

			/**
			 * Main constructor.
			 */
			SSModel():Ts(0){};
			/**
			 * Sets the internal state of the model.
			 */
			inline void setState(vector x){this->x = x; this->xk_1 = x;}

			/**
			 * State transition,
			 * Input,
			 * Model noise covariance,
			 * Model noise transformation
			 * Measurement noise covariance,
			 * Measurement noise transformation
			 */
			matrix A,B,Q,W,H,R,V,H0,R0,V0;
			/**
			 * Model sampling time
			 */
			precission Ts;

		protected:
			/**
			 * State and output vector.
			 */
			vector xk_1,x;
		};
	}
}
/* SSMODEL_HPP_ */
#endif
