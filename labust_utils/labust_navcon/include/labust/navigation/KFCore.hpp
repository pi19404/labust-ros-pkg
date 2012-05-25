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

namespace labust
{
  namespace navigation
  {
    /**
     * This class combines models with different prediction and correction approaches.
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
      void predict(typename Base::inputref u)
      {
      	assert((this->Ts) && "KFCore: Sampling time set to zero." );
        using namespace boost::numeric::ublas;
        /**
         * Forward the model in time and update the needed matrices:
         * x_(k) = f(x(k-1),u(k),0)
         */
        this->step(u);
        /**
         * Update the innovation matrix:
         * P_(k) = A(k)*P(k-1)*A'(k) + W(k)*Q(k-1)*W'(k)
         */
        this->P = prod(this->A,typename Base::matrix(prod(this->P,trans(this->A)))) +
            //this can be optimized for constant noise models
            prod(typename Base::matrix(prod(this->W,this->Q)),trans(this->W));
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
        typename Base::matrix PH = prod(this->P,trans(this->H));
        this->innovationCov = prod(this->H,PH);
        typename Base::matrix VR = prod(this->V,this->R);
        //This can be optimized for constant R, V combinations
        this->innovationCov += prod(VR,trans(this->V));

        typename Base::matrix inverse(this->innovationCov.size1(),this->innovationCov.size2());
        labust::math::gjinverse(this->innovationCov,inverse);

        this->K = prod(PH,inverse);
        /*
         * Correct the state estimate
         *
         * x(k) = x_(k) + K(k)*(measurement - y(k));
         */
        typename Base::output_type y;
        this->estimate_y(y);
        this->innovation = y_meas - y;
        this->x += prod(this->K,this->innovation);
        /**
         * Update the estimate covariance matrix.
         * P(k) = (I - K(k)*H(k))*P(k);
         */
        typename Base::matrix IKH = typename Base::eye(this->P.size1());
        IKH -= prod(this->K,this->H);
        //P = prod(typename Model::matrix(typename Model::eye(KH.size1())-KH),P);
        this->P = prod(IKH,this->P);

        return this->x;
      }
    };
  };
}
/* KFCORE_HPP_ */
#endif
