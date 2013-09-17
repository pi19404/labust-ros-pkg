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
#ifndef MTMESSAGES_HPP_
#define MTMESSAGES_HPP_
#include <labust/tritech/tritechfwd.hpp>
#include <labust/preprocessor/mem_serialized_struct.hpp>

#include <iostream>
#include <sstream>
#include <cstdint>

namespace labust
{
	namespace tritech
	{
		/**
		 * This structure represents the Tritech HEX header.
		 *
		 *  - Each message's starting character is the @ char. (1 byte)
		 *  - Hex length written down in ASCII, ie. 002A translates to 0x30 0x30 0x32 0x41 binary (4 bytes)
		 */
		struct HexHeader
		{
			enum {default_size=4};

			static inline size_t length(uint8_t* data, std::size_t len = default_size)
			{
				size_t retVal;
				std::stringstream in(std::string(reinterpret_cast<char*>(data),4));
				in>>std::hex>>retVal;
				return retVal;
			}
		};

		struct MTMsg
		{
			enum {default_size=8};
			enum {count_diff=5};

			enum mtMessage
			{
				///Null message
				mtNull = 0,
				///mtHeadData - ID:2 - data from the head
				mtHeadData = 2,
				///mtAlive - ID:4 - indicates the presence of the device
				mtAlive = 4,
				///mtReboot - ID:16 - reboots the device
				mtReboot = 16,
				///mtHeadCmd - ID:19 - head configuration
				mtHeadCmd = 19,
				///mtSendData - ID:25 - request data
				mtSendData = 25,
				///mtGGAData - ID:36 - GPGGA GPS data
				mtGPSData = 36,
				///mtAMNav - ID:52 - USBL processed data
				mtAMNavData = 52,
				///mtAMNavRaw - ID:77 - raw USBL data
				mtAMNavRaw = 77,
				///mtMiniModemCmd - ID:78 - message for the mini modem
				mtMiniModemCmd = 78,
				///mtMiniModemData - ID:79 - reply message from the mini modem
				mtMiniModemData,
				///mtMiniModemData - ID:80 - attitude sensor command message
				mtMiniAttCmd,
				///mtMiniModemData - ID:81 - attitude sensor data return message
				mtMiniAttData,
				///mtAMNavDataV2 - ID:94 - the processed USBL navigation data with modem message
				mtAMNavDataV2 = 94
			};

			MTMsg():
				size(default_size),
				txNode(255),
				rxNode(255),
				byteCount(3),
				msgType(0),
				seq(128),
				node(255),
				data(new boost::asio::streambuf()){};

			/**
			 * The methods sets the byte sizes automattically on payload size.
			 */
			inline void setup()
			{
				this->size = data->size()+ default_size;
				this->byteCount = this->size-count_diff;
			}

			uint16_t size;
			uint8_t txNode, rxNode;
			uint8_t byteCount;
			uint8_t msgType;
			uint8_t seq;
			uint8_t node;

			StreamPtr data;
		};
	}
}

///Create the serializator for archives
PP_LABUST_MAKE_BOOST_SERIALIZATOR_CLEAN(labust::tritech::MTMsg,
	(uint16_t, size)
	(uint8_t, txNode)
	(uint8_t, rxNode)
	(uint8_t, byteCount)
	(uint8_t, msgType)
	(uint8_t, seq)
	(uint8_t, node))

/* MTMESSAGES_HPP_ */
#endif
