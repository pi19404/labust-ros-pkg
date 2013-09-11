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
#include <labust/tritech/MTDevice.hpp>
#include <labust/tritech/mtMessages.hpp>

#include <boost/bind.hpp>
#include <boost/regex.hpp>

using namespace labust::tritech;

MTDevice::MTDevice(const std::string& portName, uint32_t baud):
								port(io),
								ringBuffer(ringBufferSize),
								inputSer(input, boost::archive::no_header)
{
	port.open(portName);

	if (port.is_open())
	{
		using namespace boost::asio;
		port.set_option(serial_port::baud_rate(baud));
		port.set_option(serial_port::flow_control(serial_port::flow_control::none));
		port.set_option(serial_port::character_size(8));
		port.set_option(serial_port::character_size(8));
		port.set_option(serial_port::parity(serial_port::parity::none));
	    port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    			 
		std::cout<<"Is open."<<std::endl;
	}
	else
	{
		throw std::runtime_error("MTDevice: Cannot open serial port " + portName + ".");
	}

	//Start reading until header start.
	this->start_receive(Sync);
	service = boost::thread(boost::bind(&boost::asio::io_service::run,&this->io));
}

MTDevice::~MTDevice()
{
	io.stop();
	service.join();

	std::cout<<"Closed MTDevice."<<std::endl;
}

void MTDevice::start_receive(uint8_t state)
{
	switch (state)
	{
	case Sync:
		boost::asio::async_read(port,
				input.prepare(ringBufferSize),
				boost::bind(&MTDevice::onSync,this,_1,_2));
		break;
	/*case Header:
		boost::asio::async_read(port,
				input.prepare(MTMsg::default_size),
				boost::bind(&MTDevice::onHeader,this,_1,_2));
		break;*/
	}
}

void MTDevice::onSync(const boost::system::error_code& error, std::size_t bytes_transferred)
{
	if (!error)
	{
		input.commit(bytes_transferred);
		//Put the new byte on the end of the ring buffer
		if (bytes_transferred == 1)	ringBuffer.push_back(input.sbumpc());
		else input.sgetn(reinterpret_cast<char*>(ringBuffer.data()),bytes_transferred);

		if (ringBuffer[0] == '@')
		{
			size_t len = HexHeader::length(&ringBuffer[1]);

			if ((len != 0) && (len == (ringBuffer[5]+ringBuffer[6]*256)))
			{
				StreamPtr data(new boost::asio::streambuf());
	 		    //Return the length bytes
				data->sputc(ringBuffer[5]);
				data->sputc(ringBuffer[6]);
				data->pubseekoff(-2,std::ios_base::cur);
				boost::asio::async_read(port,
					//Add +1 for the carriage return at the end of each message
					data->prepare(len-2 + 0*1),
					boost::bind(&MTDevice::onHeader,this,data,_1,_2));
				return;
			}
		}
		std::cerr<<"Out of sync."<<std::endl;
		//std::cout<<"Text:";
		//for (int i=0; i<ringBuffer.size(); ++i) {std::cout<<uint32_t(ringBuffer[i])<<",";}
		//If no size match or sync byte, move by one
		ringBuffer.erase(ringBuffer.begin());
		boost::asio::async_read(port,
				input.prepare(1),
				boost::bind(&MTDevice::onSync,this,_1,_2));
	}
	else
	{
		std::cerr<<error.message()<<std::endl;
		this->start_receive(Sync);
	}
}

void MTDevice::onHeader(StreamPtr data, const boost::system::error_code& error, std::size_t bytes_transferred)
{
	if (!error)
	{
		data->commit(bytes_transferred);
		try
		{
			boost::archive::binary_iarchive dataSer(*data, boost::archive::no_header);
			MTMsgPtr msg(new MTMsg());
			//Read header data
			dataSer >> (*msg);
			//Save data
			msg->data = data;

			if (handlers.find(msg->msgType) != handlers.end()) handlers[msg->msgType](msg);

			std::cout<<"MTDevice: node:"<<int(msg->node)<<", msg:"<<int(msg->msgType)<<std::endl;
//			std::cout<<"\t Msg data:";
//
//			std::istream in(msg->data.get());
//			while (!in.eof())
//			{
//				uint8_t c;
//				in>>c;
//				std::cout<<int(c)<<",";
//			};
//			std::cout<<std::endl;
		}
		catch (std::exception& e)
		{
			std::cerr<<"MTDevice::onHeader : "<<e.what()<<std::endl;
		}
	}
	else
	{
		std::cerr<<error.message()<<std::endl;
	}
	this->start_receive(Sync);
}

/**
 * The method sends the buffer content to the device.
 */
void MTDevice::send(MTMsgPtr message)
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
  boost::asio::write(port, output.data());
  //write data
  boost::asio::write(port, message->data->data());
}
