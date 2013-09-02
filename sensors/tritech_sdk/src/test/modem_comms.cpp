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
#include <labust/tritech/MTDevice.hpp>
#include <labust/tritech/TCPDevice.hpp>
#include <labust/tritech/mtMessages.hpp>
#include <labust/tritech/mmcMessages.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <sstream>


labust::tritech::MTMsgPtr setReplyBits(int i)
{
	using namespace labust::tritech;
	MTMsgPtr msg(new MTMsg());

	msg->txNode = 255;
	msg->rxNode = 86;
	msg->msgType = MTMsg::mtMiniModemCmd;
	msg->node=86;

	MMCMsg mmsg;
	mmsg.msgType = labust::tritech::mmcSetRepBits;
	mmsg.data[0] = i;
	mmsg.data[1] = 51;
	mmsg.data[2] = 52;
	mmsg.data[3] = 51;
	mmsg.data[4] = 57;
	mmsg.data[5] = 55;
	mmsg.data[6] = 53;

	std::ostream out();
	boost::archive::binary_oarchive ar(*msg->data, boost::archive::no_header);
	ar<<mmsg;

	return msg;
}

labust::tritech::TCONMsgPtr sendUSBL(int i)
{
	using namespace labust::tritech;
	TCONMsgPtr tmsg(new TCONMsg());
	tmsg->txNode = 255;
	tmsg->rxNode = 90;
	tmsg->node = 90;
	tmsg->msgType = MTMsg::mtMiniModemCmd;
	
	MMCMsg mmsg;
	
	switch (i)
	{
	 case 24: mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits24; break;
	 case 32: mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits32; break;
	 case 40: mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits40; break;
	 case 48: mmsg.msgType = labust::tritech::mmcGetRangeTxRxBits48; break;
     default:break;
	}

	mmsg.data[0] = i;
	mmsg.data[1] = 50;
	mmsg.data[2] = 49;
	mmsg.data[3] = 48;
	mmsg.data[4] = 55;
	mmsg.data[5] = 56;
	mmsg.data[6] = 57;

	std::ostream out();
	boost::archive::binary_oarchive ar(*tmsg->data, boost::archive::no_header);
	ar<<mmsg;

	return tmsg;
}

int main(int argc, char* argv[])
try
{
	using namespace labust::tritech;
	MTDevice device(argv[1],57600);

	/*MTMsgPtr msg(new MTMsg());

	msg->txNode = 255;
	msg->rxNode = 86;
	msg->msgType = MTMsg::mtMiniModemCmd;
	msg->node=86;

	std::ostream out(msg->data.get());

	char mmsg[]={labust::tritech::mmc,2,0,0,0,0,4,64};
	out.write(mmsg,sizeof(mmsg));
	std::stringstream str2("");
	msg->setup();
	boost::archive::binary_oarchive ar3(str2, boost::archive::no_header);
	ar3<<*msg<<mmsg;

	for(int i=0; i<str2.str().size(); ++i)
	{
		std::cout<<int(uint8_t(str2.str()[i]))<<",";
	}

	exit(0);


	//float TEST(a(2));
	
	TCONMsgPtr tmsg(new TCONMsg());
	tmsg->txNode = 255;
	tmsg->rxNode = 90;
	tmsg->node = 90;
	tmsg->msgType = MTMsg::mtMiniModemCmd;
	char mmsg2[]={mmcGetRangeTxRxByte,0,0,2,0,0,int8_t(226),64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	std::ostream tout(tmsg->data.get());
	tout.write(mmsg2,sizeof(mmsg2));
	*/
	//TCPDevice tcpdevice("127.0.0.1",4000);
	device.send(setReplyBits(48));

	char c = 0;
	while (c != 'q')
	{
		std::cin>>c;	

		if (c=='s')
		{
			std::cin>>c;

			if (c=='1') device.send(setReplyBits(24));
			if (c=='2') device.send(setReplyBits(32));
			if (c=='3') device.send(setReplyBits(40)); 
			if (c=='4') device.send(setReplyBits(48));
		}
		if (c=='u') 
		{
			std::cin>>c;

			//if (c=='1') tcpdevice.send(sendUSBL(24));
			//if (c=='2') tcpdevice.send(sendUSBL(32));
			//if (c=='3') tcpdevice.send(sendUSBL(40));
			//if (c=='4') tcpdevice.send(sendUSBL(48));
		}
	};

	//std::cin>>c;
/*
	std::stringstream str("");

	labust::tritech::MTMsg msg2;

	msg2.size = 500;
	msg2.rxNode = 10;
	msg2.txNode = 20;

	boost::archive::binary_oarchive ar(str, boost::archive::no_header);
	ar<<msg2;

	for(int i=0; i<str.str().size(); ++i)
	{
		std::cout<<int(uint8_t(str.str()[i]))<<",";
	}
	*/
	return 0;
}
catch (std::exception& e)
{
	std::cout<<"Closing gracefully due to following error: "<<e.what()<<std::endl;
}



