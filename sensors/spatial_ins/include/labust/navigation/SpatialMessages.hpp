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
 *  Author: Gyula Nagy
 *  Created: 14.11.2013.
 *********************************************************************/
#ifndef SPATIALMESSAGES_HPP_
#define SPATIALMESSAGES_HPP_
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <boost/serialization/base_object.hpp>
#include <cstdint>

typedef double vec3d[3];
typedef float vec3f[3];

namespace labust
{
	namespace spatial
	{
		struct ID
		{
			enum {
				AckPacket = 0,
				RebootPacket = 5,
				SystemState = 20,
				VelocityStdDev = 25,
				EulerStdDev = 26,
				PacketTimerPeriod = 180,
				PacketsPeriod = 181,
				SensorRanges = 184
			};
		};

		struct LEN
		{
			enum {
				AckPacket = 4,
				RebootPacket = 4,
				SystemState = 100,
				VelocityStdDev = 12,
				EulerStdDev = 12,
				PacketTimerPeriod = 4,
				PacketsPeriod = 7,
				SensorRanges = 4
			};
		};
	}
}

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(
		(labust)(spatial),
		SystemState,
		(uint16_t, systemStatus)
		(uint16_t, filterStatus)
		(uint32_t, unixTime)
		(uint32_t, microseconds)
		(vec3d, latLonHeight)
		(vec3f, linearVelocity)
		(vec3f, bodyAcc)
		(float, gravity)
		(vec3f, orientation)
		(vec3f, angularVelocity)
		(vec3f, latLonHeightStdDev))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(
		(labust)(spatial),
		Vec3fPacket,
		(vec3f, stdDev))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(
		(labust)(spatial),
		PacketTimerPeriod,
		(uint8_t, permanent)
		(uint8_t, UTCSync)
		(uint16_t, packetTimerPeriod))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(
		(labust)(spatial),
		PacketsPeriod,
		(uint8_t, permanent)
		(uint8_t, clear)
		(uint8_t, packetID)
		(uint32_t, period))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(
		(labust)(spatial),
		AckPacket,
		(uint8_t, id)
		(uint16_t, CRC)
		(uint8_t, res))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN(
		(labust)(spatial),
		SensorRanges,
		(uint8_t, permanent)
		(uint8_t, acc)
		(uint8_t, gyro)
		(uint8_t, mag))

//SPATIALMESSAGES_HPP_
#endif
