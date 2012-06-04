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
#ifndef CONTROLLERINTERFACE_HPP_
#define CONTROLLERINTERFACE_HPP_
#include <labust/apps/AppInterface.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>
#include <labust/control/controlfwd.hpp>

namespace labust
{
	namespace control
	{
		/**
		 * The generic controller interface.
		 */
		class Driver : public virtual labust::apps::App
		{
		public:
			/**
			 * Generic virtual destructor.
			 */
			virtual ~Driver(){};
			/**
			 * The method calculate the control based on the current and desired states.
			 * Returns the tau vector values.
			 *
			 * \param stateRef The desired state reference.
			 * \param state The current system state.
			 * \param tau The address of the desired TAU vector.
			 */
			virtual void getTAU(const labust::vehicles::stateMapRef stateRef,
					const labust::vehicles::stateMapRef state, const labust::vehicles::tauMapRef tau) = 0;
		};
	};
};

/* CONTROLLERINTERFACE_HPP_ */
#endif
