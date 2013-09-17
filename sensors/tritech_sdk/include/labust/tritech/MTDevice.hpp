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
#ifndef MTDEVICE_HPP_
#define MTDEVICE_HPP_
#include <labust/tritech/tritechfwd.hpp>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <string>

namespace labust
{
	namespace tritech
	{
		/**
		 * Serial port Tritech devices communication layer.
		 *
		 * \todo
		 */
		class MTDevice
		{
			enum {Sync,Header,Data};
			enum {ringBufferSize = 7};
		public:
			/**
			 * The functor typedef.
			 */
			typedef boost::function<void (MTMsgPtr)> FunctorType;
			/**
			 * The handler typedef.
			 */
			typedef std::map<int,FunctorType> HandlerMap;
			/**
			 * Default constructor with specified portname and baud rate.
			 */
			MTDevice(const std::string& portName, uint32_t baud);
			/**
			 * Generic destructor.
			 */
			~MTDevice();

			/**
			 * The method sends the buffer content to the device.
			 */
			void send(MTMsgPtr message);
			/**
			 * The method registers the handler.
			 */
			inline void registerHandlers(const HandlerMap& map)
			{
				this->handlers.insert(map.begin(),map.end());
			}


		private:
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
			boost::asio::serial_port port;
			/**
			 * The input buffer.
			 */
			boost::asio::streambuf input;
			/**
			 * The header ring buffer.
			 */
			std::vector<uint8_t> ringBuffer;
			/**
			 * Structure serializator.
			 */
			boost::archive::binary_iarchive inputSer;
			/**
			 * The runner thread.
			 */
			boost::thread service;
			/**
			 * The handler map.
			 */
			HandlerMap handlers;
		};
	}
}
/* MTDEVICE_HPP_ */
#endif
