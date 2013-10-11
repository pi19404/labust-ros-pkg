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
#include <labust/tritech/detail/message_preprocess.hpp>

#include <boost/preprocessor/tuple/rem.hpp>
#include <boost/scoped_ptr.hpp>

#include <iostream>
#include <bitset>

#include <string>
#include <map>
#include <stdexcept>

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
		template <> void LatLon2Bits::latlonToBits<10>(double lat, double lon);
		template <> void LatLon2Bits::latlonToBits<14>(double lat, double lon);
		template <> void LatLon2Bits::latlonToBits<18>(double lat, double lon);
		template <> void LatLon2Bits::latlonToBits<22>(double lat, double lon);

		/**
		 * The diver message list. Keep in mind that the final string version of the message contains
		 * the bit number as the first byte. The bytes are flipped due to little-big endian issues.
		 * Additional dynamic or static message sets can be utilized by the DiverMsg union by defining
		 * structures similar to AutoDiver, AutoTopside that will provide the necessary collection of
		 * bitmasks - extension feature.
		 *
		 * \todo Document extension feature
		 * \todo Add auto-load messages from ROS.
		 * \todo Separate the diver and topside messages into separate subclasses.
		 */
		struct DiverMsg
		{
			/**
			 * The maximum length of the message.
			 */
			enum {msgByteCount = 6};
			/**
			 * Default messages enum
			 */
			enum {nextKMLSet=6, endOfKML=7, initReq=8};
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

			typedef std::vector<int> BitMap;
			typedef std::map<int, BitMap > BitsCollection;
			static BitsCollection topsideMap, diverMap;

			DEFINE_MAPPED_MESSAGES(topsideMap,
//								  name,		code, (z,lat,lon,def,msg,kmlno,kmlx,kmly,img,emp,chk)
					(PositionInit,			0, 	(0, 22, 22, 0,  0,	0, 		0, 		0, 	0,	0, 	0))
					(Position_18,				1,	(7,	18,	18,	0,	0,	0,		0,		0,	0,	1,	0))
					(PositionMsg,				2,	(7,	 7,	7, 	0, 18,	0,		0,		0,	0,	5,	0))
					(PositionImg,				3,	(7,	 7,	0,	0, 	0,	0,		0,		0, 23,	0,	0))
					(Position_14Def,		4,	(7,	14,	14, 5,	0,	0,		0,		0,	0,	4,	0))
					(PositionMsgDef,		5,	(7,	 7,	 7, 5, 18,	0,		0,		0,	0,	0, 	0))
					(PositionImgDef,		6,	(7,	 7,	 7,	5,	0,	0,		0,		0, 18,	0,	0))
					(PositionKml,				7,	(7,	 7,	 7,	0,	0,	3,	 10,	 10,	0,	0,	0))
					(PositionChk,				8,	(7,	 7,  7,	0,	0,	0,		0,		0,	0, 17,	6))
					(PositionMsgChk,		9,	(7,	 7,	 7,	0, 12,	0,		0,		0,	0,	5,	6))
					(PositionImgChk,		10,	(7,	 7,	 7, 0,	0,	0,		0,		0, 17,	0,	6))
					(PositionDefChk,		11,	(7,	 7,	 7,	5,	0,	0,		0,		0,	0, 12,	6))
					(PositionMsgDefChk,	12,	(7,	 7,	 7, 5,	12,	0,		0,		0,	0,	0,	6))
					(PositionImgDefChk,	13,	(7,	 7,	 7, 5,	0,	0,		0,		0, 12,	0,	6)));

			DEFINE_MAPPED_MESSAGES(diverMap,
//										name,		code, (z,lat,lon,def,msg,kmlno,kmlx,kmly,img,emp,chk)
					(PositionInitAck,	 		0,	(0,	22,	22,	0,	0,	0,		0,		0,	0,	0,	0))
					(MsgReply,					 	1,	(0,	0,	0,	0, 42,	0,		0,		0,	0,	2,	0))
					(ImgReply,					 	2,	(0,	0,	0,	0, 	0,	0,		0,		0, 44,	0,	0))
					(DefReply,					 	3,	(0,	0,	0,	5, 	0,	0,		0,		0,	0, 39,	0))
					(MsgDefReply,				 	4,	(0,	0,	0,	5, 36,	0,		0,		0,	0,	3,	0))
					(ImgDefReply,				 	5,	(0,	0,	0,	5, 39,	0,		0,		0,	0,	0,	0))
					(MsgChkReply,				 	6,	(0,	0,	0,	0, 36,	0,		0,		0,	0,	2,	6))
					(ImgChkReply,				 	7,	(0,	0,	0,	0, 	0,	0,		0,		0, 38,	0,	6))
					(KmlReply,					 11,	(0,	0,	0,	0, 	0,	3,	 10,	 10, 	0, 21,	0)));

			DiverMsg(bool default_topside = true):
				latitude(0),
				longitude(0),
				depth(0),
				depthRes(0.5),
				data(num_elements,0),
				default_topside(true)
			{
				DiverMsg::register_topsideMap();
				DiverMsg::register_diverMap();
			};

			struct AutoDiver
			{
				inline static const DiverMsg::BitsCollection& map(){return DiverMsg::diverMap;};
			};

			struct AutoTopside
			{
				inline static const DiverMsg::BitsCollection& map()	{return DiverMsg::topsideMap;};
			};

			template <class MsgType>
			uint64_t encode(int msg_type = -1)
			{
				if (msg_type != -1) data[type] = msg_type;
				//assert(MsgType::map().find(data[type]) != MsgType::map().end());
				if (MsgType::map().find(type) == MsgType::map().end())
				{
				  throw std::invalid_argument("DiverMsg::encode : Unknown message type, skipping encoding.");
				}

				const BitMap& map =MsgType::map().at(data[type]);
				llEncoder.convert(latitude,longitude, map[lat]);
				data[lat] = llEncoder.lat;
				data[lon] = llEncoder.lon;
				data[z] = int(depth/depthRes);
				return labust::tools::BitPacker::pack(map, data);
			}

			inline uint64_t encode(int type = -1)
			{
				return (default_topside?encode<AutoTopside>(type):encode<AutoDiver>(type));
			}

			template <class MsgType>
			std::string toString(int type = -1)
			{
				std::string retVal(msgByteCount+1,'\0');
				retVal[0] = msgByteCount*8;
				uint64_t data = encode<MsgType>(type);
				const char* p=reinterpret_cast<const char*>(&data);
				//Flip bytes
				for (int i=0; i<msgByteCount; ++i) retVal[i+1] = p[(msgByteCount-1)-i];
				return retVal;
			}

			inline std::string toString(int type = -1)
			{
				return (default_topside?toString<AutoTopside>(type):toString<AutoDiver>(type));
			}

			template <class MsgType>
			void decode(uint64_t data, int type = -1)
			{
				if (type == -1) type = testType(data);
				//assert(MsgType::map().find(data[type]) != MsgType::map().end());
				if (MsgType::map().find(type) == MsgType::map().end())
				{
				  throw std::invalid_argument("DiverMsg::decode : Unknown message type, skipping decoding.");
				}
				labust::tools::BitPacker::unpack(data,MsgType::map().at(type), this->data);
			}

			inline void decode(uint64_t data, int type = -1)
			{
				default_topside?decode<AutoDiver>(data,type):decode<AutoTopside>(data,type);
			}

			template <class MsgType>
			void fromString(const std::string& msg, int type = -1)
			{
				if (msg.size() <msgByteCount)
				{
				  throw std::invalid_argument("DiverMsg::fromString : Wrong message size.");
				}
				uint64_t data;
				char* ret=reinterpret_cast<char*>(&data);
				/*std::cout<<"Bytes:";*/
				for (int i=0; i<msgByteCount; ++i)
				{
				 
				 ret[i] = msg[1+(msgByteCount-1)-i];
				 //std::cout<<std::bitset<8>(ret[i])<<" ";
				}				
				/*std::cout<<std::endl;*/
				decode<MsgType>(data, type);
			}

			void fromString(const std::string& msg, int type = -1)
			{
				default_topside?fromString<AutoDiver>(msg,type):fromString<AutoTopside>(msg,type);
			}

			static inline uint8_t testType(const std::string& data)
			{
				return (data[1] >> 4) & 0xF;
			}

			static inline uint8_t testType(uint64_t data, size_t msgSize = 48)
			{
				return (data >> (msgSize - 4)) & 0xF;
			}

			std::vector<int64_t> data;
			LatLon2Bits llEncoder;
			double latitude, longitude, depth, depthRes, kml_latitude, kml_longitude;
			bool default_topside;
		};

#include <labust/tritech/detail/message_preprocess_undef.hpp>
	}
}
/* DIVERMSG_HPP_ */
#endif



