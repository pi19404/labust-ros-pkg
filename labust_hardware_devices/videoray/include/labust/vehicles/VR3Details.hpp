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
#ifndef VR3DETAILS_HPP_
#define VR3DETAILS_HPP_
#include <labust/vehicles/VRComms.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The class implements the serial communication with the VideoRay Pro3 vehicle.
		 * Physical media: RS232, baud rate 9600, 8 bit, 1 stop, no parity.
     *
     * We define the following protocol (decimal numbers):
     *  Information in the 8 Bytes the PC Sends
     *        1) 35 (for all VideoRay models)
     *        2) 49 (for Pro III)
     *        3) Current for the port thruster, minimum 0, maximum 220;
     *        4) Current for the starboard thruster, minimum 0, maximum 220;
     *        5) Current for the vertical thruster, minimum 0, maximum 220;
     *        6) Current for the lights, minimum, maximum 200;
     *        7) Bit level Controls for the manipulators and auto depth,
     *        8) Bit level controls of camera tilt and focus, the direction of the thrusters,
     *  Information of the 7 Bytes VideoRay Sends
     *  The first 3 bytes of the 7 bytes contain an identifier then compass low byte,
     *  compass high byte, pressure low byte and pressure high byte.
     *        1) 40 (All VideoRay models)
     *        2) 49 (All VideoRay Pro III)
     *        3) 02 (data type for future use)
     *        4) Low byte of Orientation
     *        5) High byte of Orientation
     *        6) Low byte of Depth
     *        7) High byte of Depth
     *
     * The relation between low byte, high byte and the real value is:
     * Real value = Low byte + 256 x High Byte, for instance:
     * Orientation = Low byte of Orientation + 256 x High Byte of Orientation (0-359)
     * Depth = Low byte of Depth + 256 x High Byte of Depth (0-1023)
     *
     * When Orientation is calculated, the following conversion is needed for the real orientation:
     *
     * Real Orientation = 360 - Orientation;  // mirror the image of the orientation
     * if (Real Orientation < 90) Real Orientation = 270 + Real Orientation;
     * // shift 90 degrees counterclockwise
     * else Real Orientation = Real Orientation - 90;
		 *
		 * \todo Add input data header checking.
		 * \todo Add robusteness to faulty measurements.
		 */
		class VRSerialComms : public virtual VRComms
		{
			enum {header = 35, id = 49, inLen = 7, outLen = 8};
		public:
			/**
			 * Typedef for the bit control set.
			 */
			typedef std::bitset<16> ControlSet;
			/**
			 * Generic constructor.
			 */
			VRSerialComms();
			/**
			 * Generic destructor.
			 */
			~VRSerialComms();

			/**
			 * \override labust::vehicles::VRComms::encode
			 */
			bool encode(const ThrustVec& thrust);
			/**
			 * \override labust::vehicles::VRComms::decode
			 */
			bool decode(labust::vehicles::stateMapPtr state);

		private:
			/**
			 * The control bits set.
			 */
			ControlSet set;
		};
	}
}
/* VR3DETAILS_HPP_ */
#endif
