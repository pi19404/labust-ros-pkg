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
#ifndef VRCOMMS_HPP_
#define VRCOMMS_HPP_
#include <labust/vehicles/vehiclesfwd.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include <bitset>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The interface class for the VideoRay communication.
		 */
		class VRComms
		{
		public:
			/**
			 * The virtual destructor.
			 */
			virtual ~VRComms(){};
			/**
			 * Typedef for the thrust vector.
			 */
			typedef boost::array<int, 4> ThrustVec;
			/**
			 * Thruster enumerations.
			 */
			enum {port = 0, stbd, vert, light};
			/**
			 * The method encodes the given into the output buffer.
			 *
			 * \param thrust The given thrust values
			 * \param set The control bits.
			 */
			virtual bool encode(const ThrustVec& thrust) = 0;
			/**
			 * The method decodes the data in the buffer and returns the stateMap.
			 *
			 * \param state The state map to be filled.
			 */
			virtual bool decode(labust::vehicles::stateMapPtr state) = 0;

			/**
			 * The input buffer.
			 */
			std::vector<unsigned char> inputBuffer;
			/**
			 * The output buffer.
			 */
			std::vector<unsigned char> outputBuffer;
		};

		typedef boost::shared_ptr<VRComms> VRCommsPtr;
	}
}

/* VRCOMMS_HPP_ */
#endif
