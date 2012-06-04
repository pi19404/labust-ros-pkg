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
#ifndef NAVDRIVER_HPP_
#define NAVDRIVER_HPP_
#include <labust/navigation/navfwd.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>
#include <labust/apps/AppInterface.hpp>

namespace labust
{
	namespace navigation
	{
		/**
		 * The generic navigation interface to be used for abstracting navigation algorithms.
		 */
		class Driver : public virtual labust::apps::App
		{
		public:
			/**
			 * Generic virtual destructor.
			 */
			virtual ~Driver(){};

			/**
			 * The method performs the prediction step based on the given force input vector.
			 *
			 * \param tau The input forces and torques vector.
			 */
			virtual void prediction(const labust::vehicles::tauMapRef tau) = 0;
			/**
			 * The method performs the correction step of the filter based on supplied state measurement
			 * vector.
			 *
			 * \param measurement The measured states.
			 * \param stateHat The estimated state return.
			 */
			virtual void correction(const labust::vehicles::stateMapRef measurement, labust::vehicles::stateMapRef stateHat) = 0;
		};
	}
}


/* NAVDRIVER_HPP_ */
#endif
