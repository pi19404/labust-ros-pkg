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
#ifndef VR4DETAILS_HPP_
#define VR4DETAILS_HPP_
#include <labust/vehicles/VRComms.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * This class implements the VideoRay Futaba protocol.
		 * It consists of a 7 byte header, payload data and a checksum.
		 *
		 * The bytes are following:
		 *  1. Start of Packet 1
		 *  2. Start of Packet 2
		 *  3. Network ID - device id, 0x01 for VideoRay Pro4
		 *  4. Flags - a bit field to designate the type of response requested, 0x03 for full videoray info
		 *  5. CSR address - 0x00 in device specific response
		 *  6. Length - number of payload bytes
		 *  7. Header Checksum
		 *  8. Payload data - any data
		 *  9. Total checksum
		 *
		 *  Bytes 1 and 2 are sync bytes. Request messages have 0xFAAF and reply messages have 0xFDDF.
		 *
		 *  For better insight try:
		 *    The protocol is defined here:
		 *     http://download.videoray.com/developer/docs/PRO4_Communication_Protocol.pdf
		 *
		 *    The ROV memory map is defined here:
		 *     http://download.videoray.com/developer/docs/PRO4_CSR_memory_map.xls
		 *
		 *  \todo Test checksums and header values for incoming data.
		 *
		 */
		struct VRFutaba : public virtual VRComms
		{
			enum {inLen = 19, outLen = 14};
		public:
			/**
			 * Typedef for the bit control set.
			 */
			typedef std::bitset<16> ControlSet;
			/**
			 * Generic constructor.
			 */
			VRFutaba();
			/**
			 * Generic destructor.
			 */
			~VRFutaba();

			/**
			 * \override labust::vehicles::VRComms::encode
			 */
			bool encode(const ThrustVec& thrust);
			/**
			 * \override labust::vehicles::VRComms::decode
			 */
			bool decode(labust::vehicles::stateMapPtr state);
		};
	}
}
/* VR4DETAILS_HPP_ */
#endif
