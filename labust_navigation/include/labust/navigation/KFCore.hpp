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
#ifndef KFCORE_HPP_
#define KFCORE_HPP_
#include <labust/navigation/KFBase.hpp>

#include <vector>
#include <cassert>

///\todo Remove this after debugging
#include <iostream>

namespace labust
{
	namespace navigation
	{
		/**
		 * This class combines models with different prediction and correction approaches.
		 * \todo Switch to dynamic polymorphism for easier multi-model execution ?
		 */
		template <class Model>
		class KFCore : public KFBase< Model >
		{
			typedef KFBase< Model > Base;
		public:
			/**
			 * Generic constructor.
			 */
			KFCore(){};

			/**
			 * Perform the prediction step based on the system input.
			 *
			 * \param u Input vector.
			 */
			void predict(typename Base::inputref u = typename Model::input_type())
			{
				assert((this->Ts) && "KFCore: Sampling time set to zero." );
				/**
				 * Forward the model in time and update the needed matrices:
				 * x_(k) = f(x(k-1),u(k),0)
				 */
				this->step(u);
				/**
				 * Update the innovation matrix:
				 * P_(k) = A(k)*P(k-1)*A'(k) + W(k)*Q(k-1)*W'(k)
				 */
				this->P = this->A*this->P*this->A.transpose() +
						//this can be optimized for constant noise models
						this->W*this->Q*this->W.transpose();
			}
			/**
			 * State estimate correction based on the available measurements.
			 * Note that the user has to handle the measurement processing
			 * and populate the y_meas vector.
			 *
			 * \return Corrected state estimate.
			 */
			typename Base::vectorref correct(typename Base::outputref y_meas)
			{
				/**
				 * Calculate the Kalman gain matrix
				 *
				 * K(k) = P_(k)*H(k)'*inv(H(k)*P(k)*H(k)' + V(k)*R(k)*V(k)');
				 */
				typename Base::matrix PH = this->P*this->H.transpose();
				this->innovationCov = this->H*PH;
				typename Base::matrix VR = this->V*this->R;
				//This can be optimized for constant R, V combinations
				this->innovationCov += VR*this->V.transpose();
				this->K = PH*this->innovationCov.inverse();
				/*
				 * Correct the state estimate
				 *
				 * x(k) = x_(k) + K(k)*(measurement - y(k));
				 */
				typename Base::output_type y;
				this->estimate_y(y);
				this->innovation = y_meas - y;
				this->x += this->K*this->innovation;
				/**
				 * Update the estimate covariance matrix.
				 * P(k) = (I - K(k)*H(k))*P(k);
				 */
				typename Base::matrix IKH = Base::matrix::Identity(this->P.rows(), this->P.cols());
				IKH -= this->K*this->H;
				this->P = IKH*this->P;

				return this->xk_1 = this->x;
			}

			/**
			 * Update the matrices V and H to accommodate for available measurements.
			 * Perform per element outlier rejection and correct the state with the
			 * remaining validated measurements.
			 *
			 * \return Corrected state estimate
			 */
			template <class NewMeasVector>
			typename Base::vectorref correct(
					typename Base::outputref measurements,
					NewMeasVector& newMeas,
					bool reject_outliers = true)
			{
				//Check invariant.
				assert(measurements.size() == Model::stateNum &&
						newMeas.size() == Model::stateNum);

				std::vector<size_t> arrived;
				std::vector<typename Model::numericprecission> dataVec;

				for (size_t i=0; i<newMeas.size(); ++i)
				{
					//Outlier rejection
					if (newMeas(i) && reject_outliers)
					{
						double dist=fabs(this->x(i) - measurements(i));
						newMeas(i) = (dist <= sqrt(this->P(i,i)) + sqrt(this->R0(i,i)));

						///\todo Remove this after debugging
						if (!newMeas(i))
						{
							std::cerr<<"Outlier rejected: x(i)="<<this->x(i);
							std::cerr<<", m(i)="<<measurements(i);
							std::cerr<<", r(i)="<<sqrt(this->P(i,i)) + sqrt(this->R0(i,i));
						}
					}

					if (newMeas(i))
					{
						arrived.push_back(i);
						dataVec.push_back(measurements(i));
						newMeas(i) = 0;
					}
				}

				typename Base::output_type y_meas(arrived.size());
				this->H = Model::matrix::Zero(arrived.size(),Model::stateNum);
				this->V = Model::matrix::Zero(arrived.size(),Model::stateNum);

				for (size_t i=0; i<arrived.size();++i)
				{
					y_meas(i) = dataVec[i];
					this->H.row(i) = this->H0.row(arrived[i]);
					this->V.row(i) = this->V0.row(arrived[i]);
				}

				return this->correct(y_meas);
			}
		};
	};
}
/* KFCORE_HPP_ */
#endif
