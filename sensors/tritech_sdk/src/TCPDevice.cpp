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
 *   Date: 20.10.2010.
 *********************************************************************/
#include <labust/tritech/TCPDevice.hpp>
#include <labust/tritech/mtMessages.hpp>

#include <labust/tritech/USBLMessages.hpp>

#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <fstream>

using namespace labust::tritech;

TCPDevice::TCPDevice(const std::string& address, uint32_t port, 
	uint8_t device, uint8_t app_class, uint8_t priority):
										socket(io),
										address(address),
										port(port),
										ringBuffer(ringBufferSize),
  device(device),
  app_class(app_class),
  priority(priority)
{
	this->_setup();

	//Start reading until header start.
	this->start_receive(Sync);
	service = boost::thread(boost::bind(&boost::asio::io_service::run,&this->io));
}

TCPDevice::~TCPDevice()
{
	registerDevice(false);
	io.stop();
	service.join();
	socket.close();

	std::cout<<"Closed TCPDevice."<<std::endl;
}

void TCPDevice::_setup()
try
{
	//Resolve the Hostname and service
	boost::asio::ip::tcp::resolver resolver(io);
	std::stringstream portStr;
	portStr<<port;
	boost::asio::ip::tcp::resolver::query query(address, portStr.str());
	//Take the first
	boost::asio::ip::tcp::endpoint endpoint = *resolver.resolve(query);
	std::cout<<endpoint.address().to_string()<<", "<<address<<":"<<portStr.str()<<std::endl;
	socket.connect(endpoint);
	///Added on 17.06.2013. to avoid waiting for seanet to start.
	///\todo Refactor this code.
	while (!socket.is_open())
	{
		socket.close();
		socket.connect(endpoint);
		usleep(1000*1000);
	}

	if (socket.is_open())
	{
		registerDevice(true);
	}
	else
		throw std::runtime_error("TCPDevice::_setup : Socket not open.");
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
	throw std::runtime_error("TCPDevice::_setup : error while connecting to host.");
}

void TCPDevice::registerDevice(bool attach)
{
	///\todo Temporary added registrations
		boost::asio::streambuf output;
		std::ostream out(&output);

		TCPRequest req;
		req.app_class = app_class;
		req.node =device;
		req.priority = priority;

		if (attach)
		{
			std::cout<<"Attach to node "<<int(req.node)<<std::endl;
			req.command = TCPRequest::scAttachToNode;
		}
		else
		{
			std::cout<<"Detach from node "<<int(req.node)<<std::endl;
			req.command = TCPRequest::scDetachFromNode;
		}

		//For seanet TCP/IP command
		out<<'#';
		out.width(4);
		out.fill('0');
		out<<std::uppercase<<std::hex<<req.size;
		boost::archive::binary_oarchive dataSer(output, boost::archive::no_header);
		dataSer << req;

		boost::asio::write(socket,output.data());
}

void TCPDevice::start_receive(uint8_t state)
{
	switch (state)
	{
	case Sync:
		boost::asio::async_read(socket,
				input.prepare(ringBufferSize),
				boost::bind(&TCPDevice::onSync,this,_1,_2));
		break;
		//case Header:
		//	boost::asio::async_read(port,
		//			input.prepare(MTMsg::default_size),
		//			boost::bind(&MTDevice::onHeader,this,_1,_2));
		//	break;
	}
}

void TCPDevice::onSync(const boost::system::error_code& error, std::size_t bytes_transferred)
{
	if (!error)
	{
		input.commit(bytes_transferred);
		//Put the new byte on the end of the ring buffer
		if (bytes_transferred == 1)	ringBuffer.push_back(input.sbumpc());
		else input.sgetn(reinterpret_cast<char*>(ringBuffer.data()),bytes_transferred);

		if (ringBuffer[0] == '@' || ringBuffer[0] == '#')
		{
			size_t len = HexHeader::length(&ringBuffer[1]);
			uint16_t binLength;
			memcpy(&binLength, &ringBuffer[5],sizeof(uint16_t));

			//std::cout<<"Size of hex header:"<<len<<std::endl;
			//std::cout<<"Size of binary:"<<binLength<<std::endl;

			//This does not work for multisequences ?
			//Maybe read the whole header into the ringBuffer and then see if things
			//are multisequence or not
			if ((len != 0) && (len == (binLength)))
			{
				if (ringBuffer[0] == '@')
				{
					StreamPtr data(new boost::asio::streambuf());
					//Return the length bytes
					data->sputc(ringBuffer[5]);
					data->sputc(ringBuffer[6]);
					data->pubseekoff(-2,std::ios_base::cur);
					boost::asio::async_read(socket,
							data->prepare(len-2),
							boost::bind(&TCPDevice::onHeader,this,data,_1,_2));
					return;
				}
				else
				{
					std::cout<<"Seanet command arrived."<<std::endl;
					char* data=new char[len-2];
					boost::asio::read(socket,boost::asio::buffer(data, len-2));
					delete[] data;
					this->start_receive(Sync);
					return;
				}
			}
		}
		std::cerr<<"Out of sync:"<<std::endl;

		for(int i=0; i<ringBuffer.size(); ++i) 
			std::cerr<<int(uint8_t(ringBuffer[i]))<<",";
		std::cerr<<std::endl;

		//If no size match or sync byte, move by one
		ringBuffer.erase(ringBuffer.begin());
		boost::asio::async_read(socket,
				input.prepare(1),
				boost::bind(&TCPDevice::onSync,this,_1,_2));
	}
	else
	{
		std::cerr<<error.message()<<std::endl;
		this->start_receive(Sync);
	}
}

void TCPDevice::onHeader(StreamPtr data, const boost::system::error_code& error, std::size_t bytes_transferred)
{
	if (!error)
	{
		data->commit(bytes_transferred);
		try
		{
			boost::archive::binary_iarchive dataSer(*data, boost::archive::no_header);
			TCONMsgPtr msg(new TCONMsg());
			//Read header data
			dataSer >> (*msg);
			//Save data
			msg->data = data;

			if (handlers.find(msg->msgType) != handlers.end()) handlers[msg->msgType](msg);

			//		  //Debug stuff here
			//			if (msg->msgType == MTMsg::mtAMNavRaw)
			//			{
			//				std::cout<<"Received Raw data:"<<std::endl;
			//				std::cout<<"\tTotal size:"<<msg->data->size()<<std::endl;
			//				std::cout<<"\tMsg size:"<<msg->size<<std::endl;
			//				std::cout<<"\tMsg seq:"<<int(msg->seq)<<std::endl;
			//
			//				std::ostringstream name;
			//				name<<"rawData_"<<int(msg->seq)<<".bin";
			//				std::ofstream log(name.str());
			//				std::istream rst(msg->data.get());
			//				while (!rst.eof())
			//				{
			//					int16_t sample;
			//					rst.read(reinterpret_cast<char*>(&sample),2);
			//					log<<sample<<",";
			//				}
			//			}

			if (msg->msgType == 81) std::cout<<"Received attitude data."<<std::endl;
			if (msg->msgType == 94)
			{
				std::cout<<"Received Nav data:"<<std::endl;
				std::cout<<"Total size:"<<msg->data->size()<<std::endl;
				std::cout<<"\tMsg size:"<<msg->size<<std::endl;
				std::cout<<"\tMsg seq:"<<int(msg->seq)<<std::endl;

				//				std::cout<<"Got:"<<msg->data->size()<<std::endl;
				//				USBLDataV2 usbl_data;
				//				dataSer>>usbl_data;
				//				std::cout<<"Speed of sound:"<<int(usbl_data.nav.worldPos[2])<<std::endl;
				//				std::cout<<"Speed of sound:"<<usbl_data.nav.range<<std::endl;
				//				std::cout<<"Speed of sound:"<<usbl_data.nav.fixVOS<<std::endl;
				//				std::cout<<"Modem data:"<<int(usbl_data.modem.data[0])<<std::endl;
			}

			//			if (msg->msgType == MTMsg::mtMiniModemCmd)
			//			{
			//				std::cout<<"Received Raw data:"<<std::endl;
			//				std::cout<<"Total size:"<<msg->data->size()<<std::endl;
			//				std::cout<<"\tMsg size:"<<msg->size<<std::endl;
			//				std::cout<<"\tMsg seq:"<<int(msg->seq)<<std::endl;
			//
			//				std::istream rst(msg->data.get());
			//				while (!rst.eof())
			//				{
			//					uint8_t c;
			//					rst>>c;
			//					std::cout<<int(c)<<",";
			//				}
			//			}

			std::cout<<"TCPDevice: node:"<<int(msg->node)<<", msg:"<<int(msg->msgType)<<std::endl;
		}
		catch (std::exception& e)
		{
			std::cerr<<"TCPDevice::onHeader : "<<e.what()<<std::endl;
		}
	}
	else
	{
		std::cerr<<error.message()<<std::endl;
	}
	this->start_receive(Sync);
}

void TCPDevice::send(TCONMsgPtr message)
{
	message->setup();
	boost::asio::streambuf output;
	std::ostream out(&output);
	//prepare header
	out<<'@';
	out.width(4);
	out.fill('0');
	out<<std::uppercase<<std::hex<<message->size;
	boost::archive::binary_oarchive dataSer(output, boost::archive::no_header);
	dataSer << (*message);

	//write header
	boost::asio::write(socket, output.data());
	//write data
	boost::asio::write(socket, message->data->data());
}
