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
#ifndef TCPDEVICE_HPP_
#define TCPDEVICE_HPP_
#include <labust/tritech/tritechfwd.hpp>
#include <labust/preprocessor/mem_serialized_struct.hpp>

#include <boost/asio.hpp>
#include <boost/thread.hpp>


#include <string>
#include <map>

namespace labust
{
	namespace tritech
	{
      struct TCPRequest
      {
		  enum {default_size = 23};
		  enum client_server_command {scAttachToNode = 1, scDetachFromNode = 7};
	      enum app_classes {atNull=0, atGeneric = 15, atAMNAV = 26, atMiniAttSen = 33};
          enum con_type {rctHead = 1};

		  TCPRequest():
		  size(default_size),
			  node(labust::tritech::Nodes::USBL),
			  command(scAttachToNode),
			  priority(129),
			  app_class(atAMNAV),
			  type(rctHead){};

		uint16_t size;
        uint8_t node, command, priority, app_class, type;
        uint8_t guid[16];
      };

	  struct TCONMsg
	  {
		  enum {default_size = 14};
		  TCONMsg():
				size(default_size),
				time(0),
				txNode(255),
				rxNode(255),
				msgType(0),
				seq(128),
				node(255),
				data(new boost::asio::streambuf()){};

			/**
			 * The methods sets the byte sizes automatically on payload size.
			 */
			inline void setup()
			{
				this->size = data->size()+ default_size;
			}

			uint16_t size;
			double time;
			uint8_t txNode, rxNode;
			uint8_t msgType;
			uint8_t seq;
			uint8_t node;

			StreamPtr data;
	  };

	  /**
		 * TCP/IP Tritech devices communication layer.
		 *
		 * \todo
		 */
		class TCPDevice
		{
			enum {Sync,Header,Data};
			enum {ringBufferSize = 7};
		public:
			/**
			 * The functor typedef.
			 */
			typedef boost::function<void (TCONMsgPtr)> FunctorType;
			/**
			 * The handler typedef.
			 */
			typedef std::map<int,FunctorType> HandlerMap;

			/**
			 * Default constructor with specified portname and baud rate.
			 */
			TCPDevice(const std::string& address, uint32_t port,
				uint8_t device = Nodes::USBL,
				uint8_t app_class = TCPRequest::atAMNAV,
				uint8_t priority = 129);
			/**
			 * Generic destructor.
			 */
			~TCPDevice();

			/**
			 * The method sends the buffer content to the device.
			 */
			void send(TCONMsgPtr message);
			/**
			 * The method registers the handler.
			 */
			inline void registerHandlers(const HandlerMap& map)
			{
				this->handlers.insert(map.begin(),map.end());
			}

		private:
			/**
			 * Setup the TCP/IP connection.
			 */
			void _setup();
			/**
			 * Register the device at TCP/IP.
			 */
			void registerDevice(bool register=true);
			/**
			 * Handles synchronization.
			 */
			void onSync(const boost::system::error_code& error, std::size_t bytes_transferred);
			/**
			 * Handle next incoming header.
			 */
			void onHeader(StreamPtr data, const boost::system::error_code& error, std::size_t bytes_transferred);
			/**
			 * Handle data message.
		   */
			void onData(const boost::system::error_code& error, std::size_t bytes_transferred);
			/**
			 * Start async_reads based on current state.
			 */
			void start_receive(uint8_t state);

			/**
			 * IO service
			 */
			boost::asio::io_service io;
			/**
			 * Serial port.
			 */
			boost::asio::ip::tcp::socket socket;
			/**
			 * The remote host address.
			 */
			std::string address;
			/**
			 * The remote port.
			 */
			uint32_t port;
			/**
			 * The input buffer.
			 */
			boost::asio::streambuf input;
			/**
			 * The header ring buffer.
			 */
			std::vector<uint8_t> ringBuffer;
			/**
			 * The runner thread.
			 */
			boost::thread service;
			/**
			 * The handler map.
			 */
			HandlerMap handlers;
			/**
			 * Device registarion.
			 */
			uint8_t device,app_class,priority;
		};
	}
}

///Create the serializator for archives
PP_LABUST_MAKE_BOOST_SERIALIZATOR(labust::tritech::TCPRequest,
	(uint16_t, size)
	(uint8_t, node)
	(uint8_t, command)
	(uint8_t, priority)
	(uint8_t, app_class)
	(uint8_t, type))
	BOOST_CLASS_IMPLEMENTATION(labust::tritech::TCPRequest, boost::serialization::primitive_type)

PP_LABUST_MAKE_BOOST_SERIALIZATOR_CLEAN(labust::tritech::TCONMsg,
	(uint16_t, size)
	(double, time)
	(uint8_t, txNode)
	(uint8_t, rxNode)
	(uint8_t, msgType)
	(uint8_t, seq)
	(uint8_t, node))

/* TCPDEVICE_HPP_ */
#endif
