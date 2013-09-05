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
#include <labust/tritech/mtMessages.hpp>
#include <labust/tritech/mmcMessages.hpp>

#include <boost/asio.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <string>
#include <vector>
#include <cstdint>
#include <bitset>
#include <iostream>
#include <boost/integer/integer_mask.hpp>
#include <cassert>
#include <boost/regex.hpp>

#define ADD_DIVER_MESSAGE(NAME, CODE, DEPTHSIZE, \
	LATLONSIZE, DEFSIZE, MSGSIZE, KMLNOSIZE, KMLSIZE, IMGSIZE, VOIDSIZE, CHKSIZE) \
	struct NAME\
	{\
	  enum{type=CODE};\
		enum{depthSize=DEPTHSIZE,latlonSize=LATLONSIZE,defSize=DEFSIZE, msgSize=MSGSIZE, kmlNoSize=KMLNOSIZE, \
	  kmlSize=KMLSIZE, imgSize=IMGSIZE, voidSize=VOIDSIZE, chkSize=CHKSIZE};\
	};\


struct DiverMsg
{
	//Topside messages
	ADD_DIVER_MESSAGE(PositionInit,		0,	0,	22,	0,	0,	0,	0,	0,	0,	0);
	ADD_DIVER_MESSAGE(Position_18,		1,	7,	18,	0,	0,	0,	0,	0,	1,	0);
	ADD_DIVER_MESSAGE(PositionMsg,		2,	7,	7,	0,	23,	0,	0,	0,	0,	0);
	ADD_DIVER_MESSAGE(PositionImg,		3,	7,	7,	0,	0,	0,	0,	23,	0,	0);
	ADD_DIVER_MESSAGE(Position_14Def,	4,	7,	14,	5,	0,	0,	0,	0,	4,	0);
	ADD_DIVER_MESSAGE(PositionMsgDef,	5,	7,	7,	5,	18,	0,	0,	0,	0,	0);
	ADD_DIVER_MESSAGE(PositionImgDef,	6,	7,	7,	5,	0,	0,	0,	18,	0,	0);
	ADD_DIVER_MESSAGE(PositionKml,		7,	7,	7,	0,	0,	3,	10,	0,	0,	0);
	ADD_DIVER_MESSAGE(PositionChk,		8,	7,	7,	0,	0,	0,	0,	0,	17,	6);
	ADD_DIVER_MESSAGE(PositionMsgChk,	9,	7,	7,	0,	12,	0,	0,	0,	5,	6);
	ADD_DIVER_MESSAGE(PositionImgChk,	10,	7,	7,	0,	0,	0,	0,	17,	0,	6);
	ADD_DIVER_MESSAGE(PositionDefChk,	11,	7,	7,	5,	0,	0,	0,	0,	12,	6);
	ADD_DIVER_MESSAGE(PositionMsgDefChk,12,	7,	7,	5,	12,	0,	0,	0,	0,	6);
	ADD_DIVER_MESSAGE(PositionImgDefChk,13,	7,	7,	5,	0,	0,	0,	12,	0,	6);

	//Diver messages
	ADD_DIVER_MESSAGE(PositonInitAck,	0,	0,	22,	0,	0,	0,	0,	0,	0,	0);
	ADD_DIVER_MESSAGE(Msg,				1,	0,	0,	0,	0,	0,	0,	0,	44,	0);

	DiverMsg():
		latitude(0),
		longitude(0),
		z(0),
		depthRes(0.5){};

	//\todo Add automatic extraction of lat-lon data from double values
	template <class msg>
	uint64_t pack()
	{
		fullmsg = msg::type;
		fullmsg <<= msg::depthSize;
		fullmsg |= int(z/depthRes) & boost::low_bits_mask_t<msg::depthSize>::sig_bits;
		fullmsg <<= msg::latlonSize;
		latlonToBits<msg::latlonSize>(latitude,longitude);
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
	};

	template <size_t precission>
	std::pair<int,int> latlonToBits(double lat, double lon){return std::pair<int,int>();};

	double latitude, longitude, z, depthRes;
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
	this->lon = int((int(lon)+180)*10000 + min*100);

	min = (lat - int(lat))*60;
	this->lat = int((int(lat)+90)*10000 + min*100);

	return std::pair<int,int>(this->lat,this->lon);
}

template <>
inline std::pair<int,int> DiverMsg::latlonToBits<18>(double lat, double lon)
{
	this->lat = int((lat - int(lat))*600000)%100000;
	this->lon = int((lon - int(lon))*600000)%100000;
	return std::pair<int,int>(this->lat,this->lon);
}

template <>
std::pair<int,int> DiverMsg::latlonToBits<14>(double lat, double lon)
{
	double min = (lon - int(lon))*60;
	this->lon = int((min - int(min))*10000);

	min = (lat - int(lat))*60;
	this->lat = int((min - int(min))*10000);

	return std::pair<int,int>(this->lat,this->lon);
}

template <>
inline std::pair<int,int> DiverMsg::latlonToBits<7>(double lat, double lon)
{
	latlonToBits<14>(lat,lon);
	this->lat%=100;
	this->lon%=100;
	return std::pair<int,int>(this->lat,this->lon);
}

void sendToModem(boost::asio::serial_port& port, DiverMsg& msg)
{
	using namespace labust::tritech;
	MTMsgPtr tmsg(new MTMsg());
	tmsg->txNode = labust::tritech::Nodes::SlaveModem;
	tmsg->rxNode = labust::tritech::Nodes::Surface;
	tmsg->node = labust::tritech::Nodes::SlaveModem;
	tmsg->msgType = MTMsg::mtMiniModemData;
	MMCMsg mmsg;
	mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits48;
	mmsg.data[0] = 48;

	const char* p=reinterpret_cast<const char*>(&msg.fullmsg);
	std::cout<<"Payload:";
//	for (int i=5; i>=0; --i)
//	{
//		mmsg.data[i+1] = p[i];
//		std::cout<<int(p[i])<<",";
//	}
	for (int i=0; i<6; ++i)
	{
		mmsg.data[i+1] = p[5-i];
		std::cout<<int(p[5-i])<<",";
	}
	std::cout<<std::endl;

	std::bitset<48> pyl(msg.fullmsg);
	std::cout<<"Payload 2:"<<pyl<<std::endl;

	boost::archive::binary_oarchive ar(*tmsg->data, boost::archive::no_header);
	ar<<mmsg;

	tmsg->setup();
	boost::asio::streambuf output;
	std::ostream out(&output);
	//prepare header
  out<<'@';
  out.width(4);
  out.fill('0');
  out<<std::uppercase<<std::hex<<tmsg->size;
  boost::archive::binary_oarchive dataSer(output, boost::archive::no_header);
  dataSer << (*tmsg);

  //write header
  boost::asio::write(port, output.data());
  //write data
  boost::asio::write(port, tmsg->data->data());
}

void writeLatLon(DiverMsg& msg)
{
	double lat = msg.latitude;
	double lon = msg.longitude;
	std::cout<<"Sending lat-lon:"<<int(lat)<<"\° "<<(lat-int(lat))*60<<"', ";
	std::cout<<int(lon)<<"\° "<<(lon-int(lon))*60<<std::endl;
	std::cout<<"\t encoded lat-lon:"<<msg.lat<<", "<<msg.lon<<std::endl;
}

int main(int argc, char* argv[])
{
	boost::asio::io_service io;
	boost::asio::serial_port port(io);

	port.open("/dev/rfcomm0");
	port.set_option(boost::asio::serial_port::baud_rate(57600));
	port.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));

	//boost::asio::streambuf incoming;
	//boost::asio::read_until(port, incoming, boost::regex("\n"));

	if (port.is_open()) std::cout<<"Port is open."<<std::endl;

	uint8_t test_data[] = {0x40,0x30,0x30,0x31,0x46,0x1F,0x00,0x56,0xFF,0x1A,0x4F,0x80,0x56,0x21,0x02,0x00,
			0x00,0x00,0xB8,0x0B,0x30,0x22,0xC3,0xF3,0x38,0x78,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

	using namespace labust::tritech;

	double depth = 5;
	double lat=45.769216667, lon=16.001851667;
	//init
	DiverMsg msg;
	msg.latitude = lat;
	msg.longitude = lon;
	msg.z = depth;
	msg.msg = 0;
	msg.pack<DiverMsg::PositionInit>();
	writeLatLon(msg);
	sendToModem(port,msg);
	usleep(1000*1000);

	//7bit
	double step = 0.003/60;
	for (int i=0; i<4; ++i)
	{
		msg.latitude += step;
		msg.longitude += step;
		msg.pack<DiverMsg::PositionMsg>();
		writeLatLon(msg);
		sendToModem(port, msg);
		usleep(1000*1000);
	}
	for (int i=0; i<8; ++i)
	{
		msg.latitude -= step;
		msg.longitude -= step;
		msg.pack<DiverMsg::PositionMsg>();
		writeLatLon(msg);
		sendToModem(port, msg);
		usleep(1000*1000);
	}
	//14 bit
	/*step = 0.3/60;
	for (int i=0; i<4; ++i)
	{
		msg.latitude += step;
		msg.longitude += step;
		msg.pack<DiverMsg::PositionDef>();
		writeLatLon(msg);
		sendToModem(port, msg);
		usleep(1000*1000);
	}
	step = 0.3/60;
	for (int i=0; i<8; ++i)
	{
		msg.latitude -= step;
		msg.longitude -= step;
		msg.pack<DiverMsg::PositionDef>();
		writeLatLon(msg);
		sendToModem(port, msg);
		usleep(1000*1000);
	}*/
	//18 bit

	usleep(5000*1000);

	step = 3./60;
	for (int i=0; i<4; ++i)
	{
		msg.latitude += step;
		msg.longitude += step;
		msg.pack<DiverMsg::Position_18>();
		writeLatLon(msg);
		sendToModem(port, msg);
		usleep(1000*1000);
	}
	step = 3./60;
	for (int i=0; i<8; ++i)
	{
		msg.latitude -= step;
		msg.longitude -= step;
		msg.pack<DiverMsg::Position_18>();
		writeLatLon(msg);
		sendToModem(port, msg);
		usleep(1000*1000);
	}

	return 0;
}



