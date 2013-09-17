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
#ifndef BITPACKER_HPP_
#define BITPACKER_HPP_
#include <vector>
#include <cstdint>
#include <cassert>

//#include <bitset>
//#include <iostream>

namespace labust
{
	namespace tools
	{
		/**
		 * Bitpacker for diver messages.
		 *
		 * \todo Generalize this bit packer with boost::serialization concepts,
		 * similar to the XML approach.
		 */
		struct BitPacker
		{
			template <class BitMapType, class DataType>
			static uint64_t pack(const std::vector<BitMapType>& bitmap,
					const std::vector<DataType>& data)
			{
				uint64_t retVal(0);
				assert(bitmap.size() == data.size() &&
						"The bitmap and data size have to be the same length.");

				for (uint32_t i=0,j=1; i<bitmap.size(); ++i,++j)
				{
					DataType bitmask = (1 << bitmap[i]) -1;
					retVal |= data[i] & bitmask;
					if (j < bitmap.size()) retVal <<= bitmap[j];
				}

				return retVal;
			}

			template <class BitMapType, class DataType>
			static void unpack(uint64_t msg, const std::vector<BitMapType>& bitmap,
					std::vector<DataType>& data)
			{
				data.clear();
				data.resize(bitmap.size());
				for (int i=bitmap.size()-1; i>=0; --i)
				{
					DataType bitmask = DataType(DataType(1) << bitmap[i]) -1;
					data[i]=(msg & bitmask);
					//std::cout<<"Unpacking message:"<<std::bitset<48>(msg)<<" with bitmap: "<<std::bitset<sizeof(DataType)*8>(bitmask)<<std::endl;
					msg >>= bitmap[i];
				}
			};

			template <class BitMapType, class DataType>
			static uint64_t pack_flipped(const std::vector<BitMapType>& bitmap,
					const std::vector<DataType>& data)
			{
				uint64_t retVal(0);
				assert(bitmap.size() == data.size() &&
						"The bitmap and data size have to be the same length.");

				for (int i=bitmap.size()-1,j=bitmap.size()-2; i>=0; --i,--j)
				{
					DataType bitmask = (1 << bitmap[i]) -1;
					retVal |= data[i] & bitmask;
					if (j >= 0) retVal <<= bitmap[j];
				}

				return retVal;
			}

			template <class BitMapType, class DataType>
			static void unpack_flipped(uint64_t msg, const std::vector<BitMapType>& bitmap,
					std::vector<DataType>& data)
			{
				data.clear();
				for (uint32_t i=0; i<bitmap.size(); ++i)
				{
					DataType bitmask = (1 << bitmap[i]) -1;
					data.push_back(msg & bitmask);
					msg >>= bitmap[i];
				}
			};
		};
	}
}
/* BITPACKER_HPP_ */
#endif



