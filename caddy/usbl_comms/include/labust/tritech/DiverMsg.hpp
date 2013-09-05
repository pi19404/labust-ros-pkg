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
 *
 *  Author: Dula Nad
 *  Created: 29.04.2013.
 *********************************************************************/
#ifndef DIVERMSG_HPP_
#define DIVERMSG_HPP_
#include <labust/tools/BitPacker.hpp>

#include <bitset>
#include <iostream>

#include <boost/preprocessor/tuple/rem.hpp>

#include <string>

namespace labust
{
	namespace tritech
	{
		/**
		 * The utility structure for converting latitude, longitude diver coordinates to
		 * bit encodings.
		 *
		 * \todo Move into
		 */
		class LatLon2Bits
		{
		public:
			void convert(double lat, double lon, int bits = 7);

			int lat, lon;

		protected:
			template <size_t precission>
			void latlonToBits(double lat, double lon);
		};

		template <> void LatLon2Bits::latlonToBits<7>(double lat, double lon);
		template <> void LatLon2Bits::latlonToBits<14>(double lat, double lon);
		template <> void LatLon2Bits::latlonToBits<18>(double lat, double lon);
		template <> void LatLon2Bits::latlonToBits<22>(double lat, double lon);

#define ADD_DIVER_MESSAGE(name, code, bits) \
	struct name \
	{ \
		enum {type = code}; \
		static inline\
		const std::vector<int>& bitmap() \
		{ \
			static const std::vector<int> bitmap({4, BOOST_PP_TUPLE_REM_CTOR(11, bits)});	\
			return bitmap; \
		} \
	}; \

		/**
		 * The diver message list. Keep in mind that the final string version of the message contains
		 * the bit number as the first byte. The bytes are flipped due to little-big endian issues.
		 * \todo Add auto-load messages from ROS.
		 * \todo Add auto-unpack data.
		 * \todo Separate the diver and topside messages into separate subclasses.
		 */
		struct DiverMsg
		{
			/**
			 * The maximum length of the message.
			 */
			enum {msgByteCount = 6};
			/**
			 * The list of encoded variables.
			 */
			enum {type=0,
				z,
				lat,
				lon,
				def,
				msg,
				kmlno,
				kmlx,
				kmly,
				img,
				empty,
				chksum,
				num_elements};

			//Topside messages				name,		code, (z,lat,lon,def,msg,kmlno,kmlx,kmly,img,emp,chk)
			ADD_DIVER_MESSAGE(PositionInit,			0, 	(0, 22, 22, 0,  0,	0, 		0, 		0, 	0,	0, 	0));
			ADD_DIVER_MESSAGE(Position_18,			1,	(7,	18,	18,	0,	0,	0,		0,		0,	0,	1,	0));
			ADD_DIVER_MESSAGE(PositionMsg,			2,	(7,	 7,	7, 	0, 18,	0,		0,		0,	0,	5,	0));
			ADD_DIVER_MESSAGE(PositionImg,			3,	(7,	 7,	0,	0, 	0,	0,		0,		0, 23,	0,	0));
			ADD_DIVER_MESSAGE(Position_14Def,		4,	(7,	14,	14, 5,	0,	0,		0,		0,	0,	4,	0));
			ADD_DIVER_MESSAGE(PositionMsgDef,		5,	(7,	 7,	 7, 5, 18,	0,		0,		0,	0,	0, 	0));
			ADD_DIVER_MESSAGE(PositionImgDef,		6,	(7,	 7,	 7,	5,	0,	0,		0,		0, 18,	0,	0));
			ADD_DIVER_MESSAGE(PositionKml,			7,	(7,	 7,	 7,	0,	0,	3,	 10,	 10,	0,	0,	0));
			ADD_DIVER_MESSAGE(PositionChk,			8,	(7,	 7,  7,	0,	0,	0,		0,		0,	0, 17,	6));
			ADD_DIVER_MESSAGE(PositionMsgChk,		9,	(7,	 7,	 7,	0, 12,	0,		0,		0,	0,	5,	6));
			ADD_DIVER_MESSAGE(PositionImgChk,		10,	(7,	 7,	 7, 0,	0,	0,		0,		0, 17,	0,	6));
			ADD_DIVER_MESSAGE(PositionDefChk,		11,	(7,	 7,	 7,	5,	0,	0,		0,		0,	0, 12,	6));
			ADD_DIVER_MESSAGE(PositionMsgDefChk,12,	(7,	 7,	 7, 5,	12,	0,		0,		0,	0,	0,	6));
			ADD_DIVER_MESSAGE(PositionImgDefChk,13,	(7,	 7,	 7, 5,	0,	0,		0,		0, 12,	0,	6));

			//Diver messages						name,	code,	(z,lat,lon,def,msg,kmlno,kmlx,kmly,img,emp,chk)
			ADD_DIVER_MESSAGE(PositionInitAck,	 0,	(0,	22,	22,	0,	0,	0,		0,		0,	0,	0,	0));
			ADD_DIVER_MESSAGE(Msg,							 1,	(0,	0,	0,	0, 42,	0,		0,		0,	0,	0,	0));

			DiverMsg():
				latitude(0),
				longitude(0),
				depth(0),
				depthRes(0.5),
				data(num_elements,0){};

			template <class MsgType>
			uint64_t encode()
			{
				llEncoder.convert(latitude,longitude, MsgType::bitmap()[lat]);
				data[type] = MsgType::type;
				data[lat] = llEncoder.lat;
				data[lon] = llEncoder.lon;
				data[z] = int(depth/depthRes);
				return labust::tools::BitPacker::pack(MsgType::bitmap(), data);
			}

			template <class MsgType>
			std::string toString()
			{
				std::string retVal(msgByteCount+1,'\0');
				retVal[0] = msgByteCount*8;
				uint64_t data = encode<MsgType>();
				const char* p=reinterpret_cast<const char*>(&data);
				//Flip bytes
				for (int i=0; i<msgByteCount; ++i) retVal[i+1] = p[(msgByteCount-1)-i];
				//std::cout<<"Send:"<<std::bitset<48>(data);
				return retVal;
			}

			template <class MsgType>
			void decode(uint64_t data)
			{
				labust::tools::BitPacker::unpack(data,MsgType::bitmap(), this->data);
			}

			template <class MsgType>
			void fromString(const std::string& msg)
			{
				uint64_t data;
				char* ret=reinterpret_cast<char*>(&data);
				for (int i=0; i<msgByteCount; ++i) ret[i] = msg[1+(msgByteCount-1)-i];
				//for (int i=0; i<msgByteCount; ++i) ret[i] = msg[1+i];
				//std::cout<<"Bits:"<<std::bitset<48>(data)<<std::endl;
				decode<MsgType>(data);
			}

			static inline uint8_t testType(const std::string& data)
			{
				return data[1] & 0xF0;
			}

			static inline uint8_t testType(uint64_t data, size_t msgSize = 48)
			{
				return (data >> (msgSize - 4)) & 0xF;
			}

			std::vector<int64_t> data;
			LatLon2Bits llEncoder;
			double latitude, longitude, depth, depthRes;
		};

#undef ADD_DIVER_MESSAGE
	}
}
/* DIVERMSG_HPP_ */
#endif



