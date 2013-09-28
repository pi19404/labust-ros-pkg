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
/*********************************************************************
 * Author: Đula Nađ
 *   Date: 11.12.2012.
 *********************************************************************/
#ifndef USBLMESSAGES_HPP_
#define USBLMESSAGES_HPP_
#include <labust/tritech/mmcMessages.hpp>
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <boost/array.hpp>
#include <cstdint>      

namespace labust
{
	namespace tritech
	{
		typedef boost::array<float,5> vec5f;
		typedef boost::array<float,4> vec4f;
		typedef boost::array<float,3> vec3f;
		typedef boost::array<double,3> vec3d;
		typedef boost::array<int16_t,3> vec3int16;
	}
}
BOOST_CLASS_IMPLEMENTATION(labust::tritech::vec3f , boost::serialization::primitive_type)
BOOST_CLASS_IMPLEMENTATION(labust::tritech::vec3d , boost::serialization::primitive_type)
BOOST_CLASS_IMPLEMENTATION(labust::tritech::vec4f , boost::serialization::primitive_type)
BOOST_CLASS_IMPLEMENTATION(labust::tritech::vec5f , boost::serialization::primitive_type)
BOOST_CLASS_IMPLEMENTATION(labust::tritech::vec3int16 , boost::serialization::primitive_type)

///\todo Document the USBLData and USBLDataV2 classes.
PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(tritech),USBLData,
			(size_t, time_ms)
			(uint8_t,reply_validity)
			(vec3f, doa)
			(float, RMS)
			(float, usblAngleQuality)
			(float, usblRangeQuality)
			(vec4f, reliability)
			(vec3d, attitude)			
			(bool, isTransponder)
			(uint16_t, unitID)
			(vec3d, relativePos)
			(vec5f, sigma)
			(float, fixVOS)
			(double, range)
			(vec3d, attitudeCorrectedPos)
			(vec3d, worldPos)
			(double, time))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(tritech),AttSenData,
		 (uint8_t, cmd)
		 (uint32_t, time)
	     (vec3int16, acc)
		 (vec3int16, mag)
		 (vec3int16, gyro)
		 (int16_t, pressure)
		 (int16_t, externalTemp)
		 (int16_t, internalTemp))

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(tritech),USBLDataV2,
	(labust::tritech::USBLData, nav)
	(labust::tritech::MMCMsg, modem))

/* USBLMESSAGES_HPP_ */
#endif
