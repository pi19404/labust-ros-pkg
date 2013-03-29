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
*  Created: 14.02.2013.
*********************************************************************/
#include <string>
#include <vector>
#include <cstdint>
#include <bitset>
#include <iostream>
#include <boost/integer/integer_mask.hpp>

template <uint8_t type, size_t depthSize, size_t latlonSize, size_t msgSize>
struct DiverMsg
{
	uint64_t pack()
	{
		uint64_t result(type);
		result <<= depthSize;
		result |= depth & boost::low_bits_mask_t<depthSize>::sig_bits;
		result <<= latlonSize;
		result |= lat & boost::low_bits_mask_t<latlonSize>::sig_bits;
		result <<= latlonSize;
		result |= lon & boost::low_bits_mask_t<latlonSize>::sig_bits;
		result <<= msgSize;
		result |= payload & boost::low_bits_mask_t<msgSize>::sig_bits;
		return result;
	}

	uint64_t depth, lat, lon, payload;
};

template <size_t precission>
std::pair<int,int> latlonToBits(double lat, double lon);

template <>
inline std::pair<int,int> latlonToBits<22>(double lat, double lon)
{
	double min = (lon - int(lon))*60;
	int clon = int((int(lon)+180)*10000 + min*100);

	min = (lat - int(lat))*60;
	int clat = int((int(lat)+90)*10000 + min*100);

	return std::pair<int,int>(clat,clon);
}

template <>
inline std::pair<int,int> latlonToBits<18>(double lat, double lon)
{
	return std::pair<int,int>(int((lat - int(lat))*600000)%100000,int((lon - int(lon))*600000)%100000);
}

template <>
std::pair<int,int> latlonToBits<14>(double lat, double lon)
{
	double min = (lon - int(lon))*60;
	int clon = int((min - int(min))*10000);

	min = (lat - int(lat))*60;
	int clat = int((min - int(min))*10000);

	return std::pair<int,int>(clon,clat);
}

template <>
inline std::pair<int,int> latlonToBits<7>(double lat, double lon)
{
	std::pair<int,int> val = latlonToBits<14>(lat,lon);
	return std::pair<int,int>(val.first%100,val.second%100);
}


int main(int argc, char* argv[])
{

	double depth = 5;
	double lat=45.769216667, lon=16.101851667;

	DiverMsg<3,0,22,0> msg;

	msg.depth = depth*2;

	std::cout<<"Lat min:"<<(lat - int(lat))*60<<", lon min:"<<(lon - int(lon))*60<<std::endl;

	std::pair<int,int> latlon = latlonToBits<22>(lat,lon);
	msg.lat = latlon.first;
	msg.lon = latlon.second;

	std::cout<<"Lat enc:"<<msg.lat<<", lon enc:"<<msg.lon<<std::endl;

	//Message type
	std::bitset<4> type_enc(3);
	int msgV=0;
	char* p=reinterpret_cast<char*>(&msgV);
	p[0] = 1;
	p[1] = 2;
	p[2] = 3;
	msg.payload = msgV;

	std::bitset<48> result(msg.pack());
	std::cout<<"Result:"<<result<<std::endl;

	return 0;
}



