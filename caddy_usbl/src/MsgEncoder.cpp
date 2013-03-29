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
#include <cassert>

#define ADD_DIVER_MESSAGE(NAME, CODE, DEPTHSIZE, \
	LATLONSIZE, MSGSIZE) \
	struct NAME\
	{\
	  enum{type=CODE};\
		enum{depthSize=DEPTHSIZE,latlonSize=LATLONSIZE,msgSize=MSGSIZE};\
	};\


struct DiverMsg
{
	ADD_DIVER_MESSAGE(PositionInit,1,0,22,0);
	ADD_DIVER_MESSAGE(Position,2,7,18,1);
	ADD_DIVER_MESSAGE(PositionMsg,3,7,7,23);

	//\todo Add automatic extraction of lat-lon data from double values
	template <class msg>
	uint64_t pack()
	{
		fullmsg = msg::type;
		fullmsg <<= msg::depthSize;
		fullmsg |= depth & boost::low_bits_mask_t<msg::depthSize>::sig_bits;
		fullmsg <<= msg::latlonSize;
		fullmsg |= lat & boost::low_bits_mask_t<msg::latlonSize>::sig_bits;
		fullmsg <<= msg::latlonSize;
		fullmsg |= lon & boost::low_bits_mask_t<msg::latlonSize>::sig_bits;
		fullmsg <<= msg::msgSize;
		fullmsg |= this->msg & boost::low_bits_mask_t<msg::msgSize>::sig_bits;
		return fullmsg;
	}

	static inline uint8_t testType(uint64_t data, size_t msgSize = 48)
	{
		return (data >> (msgSize - 4)) & 0xF;
	}

	template <class msg>
	bool unpack(uint64_t data)
	{
		fullmsg=data;
		this->msg = data & boost::low_bits_mask_t<msg::msgSize>::sig_bits;
		data >>= msg::msgSize;
		lon = data & boost::low_bits_mask_t<msg::latlonSize>::sig_bits;
		data >>= msg::latlonSize;
		lat = data & boost::low_bits_mask_t<msg::latlonSize>::sig_bits;
		data >>= msg::latlonSize;
		depth = data & boost::low_bits_mask_t<msg::depthSize>::sig_bits;
		data >>= msg::depthSize;

		assert(((data & 0x0F) == msg::type) &&
				"DiverMsg desired unpack type and received data type do not match.");
	}

	template <size_t precission>
	std::pair<int,int> latlonToBits(double lat, double lon){return std::pair<int,int>();};

	double lat, lon, depth;
	uint64_t fullmsg;
	uint8_t depth;
	int lat,lon;
	uint64_t msg;
	//uint8_t noKML, def, checksum;
	//int kmlX, kmlY;
};

template <>
inline std::pair<int,int> DiverMsg::latlonToBits<22>(double lat, double lon)
{
	double min = (lon - int(lon))*60;
	int clon = int((int(lon)+180)*10000 + min*100);

	min = (lat - int(lat))*60;
	int clat = int((int(lat)+90)*10000 + min*100);

	return std::pair<int,int>(clat,clon);
}

template <>
inline std::pair<int,int> DiverMsg::latlonToBits<18>(double lat, double lon)
{
	return std::pair<int,int>(int((lat - int(lat))*600000)%100000,int((lon - int(lon))*600000)%100000);
}

template <>
std::pair<int,int> DiverMsg::latlonToBits<14>(double lat, double lon)
{
	double min = (lon - int(lon))*60;
	int clon = int((min - int(min))*10000);

	min = (lat - int(lat))*60;
	int clat = int((min - int(min))*10000);

	return std::pair<int,int>(clon,clat);
}

template <>
inline std::pair<int,int> DiverMsg::latlonToBits<7>(double lat, double lon)
{
	std::pair<int,int> val = latlonToBits<14>(lat,lon);
	return std::pair<int,int>(val.first%100,val.second%100);
}


int main(int argc, char* argv[])
{

	double depth = 5;
	double lat=45.769216667, lon=16.101851667;

	DiverMsg msg;

	msg.depth = depth*2;

	std::cout<<"Lat min:"<<(lat - int(lat))*60<<", lon min:"<<(lon - int(lon))*60<<std::endl;

	std::pair<int,int> latlon = latlonToBits<7>(lat,lon);
	msg.lat = latlon.first;
	msg.lon = latlon.second;

	std::cout<<"Lat enc:"<<msg.lat<<", lon enc:"<<msg.lon<<std::endl;

	//Message type
	std::bitset<4> type_enc(3);
	int msgV=0;
	char* p=reinterpret_cast<char*>(&msgV);
	p[0] = 48;
	p[1] = 49;
	p[2] = 50;
	msg.msg = msgV;

	std::bitset<48> result(msg.pack<DiverMsg::PositionMsg>());
	std::cout<<"Result:"<<result<<std::endl;

	DiverMsg msg2;
	msg2.unpack<DiverMsg::PositionMsg>(msg.pack<DiverMsg::PositionMsg>());
	std::cout<<"Lat dec:"<<msg2.lat<<", lon dec:"<<msg2.lon<<std::endl;
	const char* pt=reinterpret_cast<const char*>(&msg2.msg);
	std::cout<<"Received message:"<<pt[0]<<pt[1]<<pt[2]<<std::endl;

	return 0;
}



