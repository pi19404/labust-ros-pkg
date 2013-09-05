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
*  Created: 02.04.2013.
*********************************************************************/
#include <labust/tritech/DiverMsg.hpp>
#include <stdexcept>

using namespace labust::tritech;

#define PP_ADD_CASE(x) \
		case x: \
		this->latlonToBits<x>(lat,lon); \
		break; \

void LatLon2Bits::convert(double lat, double lon, int bits)
{
	switch (bits)
	{
		PP_ADD_CASE(7);
		PP_ADD_CASE(14);
		PP_ADD_CASE(18);
		PP_ADD_CASE(22);
	default:
		throw std::runtime_error("LatLon2Bits: Missing lat-lon conversion definition.");
	}
}

#undef ADD_CASE
template <>
void LatLon2Bits::latlonToBits<22>(double lat, double lon)
{
	double min = (lon - int(lon))*60;
	this->lon = int((int(lon)+180)*10000 + min*100);

	min = (lat - int(lat))*60;
	this->lat = int((int(lat)+90)*10000 + min*100);
}

template <>
void LatLon2Bits::latlonToBits<18>(double lat, double lon)
{
	this->lat = int((lat - int(lat))*600000)%100000;
	this->lon = int((lon - int(lon))*600000)%100000;
}

template <>
void LatLon2Bits::latlonToBits<14>(double lat, double lon)
{
	double min = (lon - int(lon))*60;
	this->lon = int((min - int(min))*10000);

	min = (lat - int(lat))*60;
	this->lat = int((min - int(min))*10000);
}

template <>
void LatLon2Bits::latlonToBits<7>(double lat, double lon)
{
	this->latlonToBits<14>(lat,lon);
	this->lat%=100;
	this->lon%=100;
}
