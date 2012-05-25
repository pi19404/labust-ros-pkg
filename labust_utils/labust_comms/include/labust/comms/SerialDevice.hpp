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
#ifndef SERIALDEVICE_HPP_
#define SERIALDEVICE_HPP_
#include <labust/comms/commsfwd.hpp>
#include <labust/comms/MMSerialize.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace labust
{
	namespace comms
	{
		/**
		 * This class implements the general purpose serial device decoder.
		 * \todo Add syncronized data access
		 * \todo Add support for binary data (not terminator)
		 * \todo Add terminator specification
		 */
		template <class Handler>
		class SerialDevice
		{
			typedef boost::shared_ptr<boost::thread> ThreadPtr;
		public:
			/**
			 * Main constructor. Configures the devices using the XML configuration file.
			 */
			SerialDevice(const labust::xml::ReaderPtr reader, const std::string& id):
				io(),
				port(io)
			{
				this->configure(reader,id);
				this->start_receive(port);
				threadPtr.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &this->io)));
			}
			/**
			 * Generic destructor.
			 */
			~SerialDevice()
			{
				this->io.stop();
				threadPtr->join();
			}
			/**
			 * Send XML encoded data as a string.
			 *
			 * \param str The data to be sent by the device.
			 */
			void sendData(const std::string& str);
			/**
			 * Register the callback function to call when new data is available.
			 *
			 * \param callback A function accepting and string.
			 */
			void registerCallback(const Handler::CallbackFunction& callback){this->callback = callback;};

		private:
			/**
			 * Configures the modem application.
			 */
			void configure(const labust::xml::ReaderPtr& reader, const std::string& id);
			/**
			 * The receive function start.
			 */
			void start_receive()
			{
				boost::asio::async_read_until(port, sbuffer, boost::regex("\n"),this->handler);
			}
			/**
			 * Input callback.
			 */
			void handleInput(const boost::system::error_code& error, const size_t& transferred);
			/**
			 * ASIO io service.
			 */
			boost::asio::io_service io;
			/**
			 * Serial port
			 */
			boost::asio::serial_port port;
			/**
			 * IO service runner thread.
			 */
			ThreadPtr threadPtr;
			/**
			 * Stream buffer
			 */
			boost::asio::streambuf sbuffer;
			/**
			 * Callback functor.
			 */
			CallbackFunction callback;
			/**
			 * XML string
			 */
			std::string xmlstr;
			/**
			 * Incoming and outgoing message queues.
			 */
			labust::comms::MMSerialize::msgqueue incoming, outgoing;
			/**
			 * Number of modem packets that form a message for the translator.
			 */
			size_t packetNum;
		};
	}
}

#endif /* SERIALDEVICE_HPP_ */
