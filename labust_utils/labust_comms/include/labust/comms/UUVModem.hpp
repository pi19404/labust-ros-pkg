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
#ifndef UUVMODEM_HPP_
#define UUVMODEM_HPP_
#include <labust/comms/commsfwd.hpp>
#include <labust/comms/MMSerialize.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <string>
#include <queue>

namespace labust
{
	namespace comms
	{
		/**
		 * This class implements the communication through the acoustic modem.
		 *
		 * \todo Add the XML configuration option
		 * \todo Add after encoding callback
		 * \todo Add syncronized data access
		 * \todo Add plugable translator loader
		 * \todo Rename this to iSerial like generic port reader object
		 * \todo Add support for binary data (not terminator)
		 * \todo Add terminator specification
		 */
		class UUVModem
		{
			typedef boost::function<void (const std::string&)> CallbackFunction;
		public:
			/**
			 * Main constructor. Configured through the XML configuration file.
			 */
			UUVModem(const labust::xml::ReaderPtr reader, const std::string& id);
			/**
			 * Generic destructor.
			 */
			~UUVModem();

			/**
			 * Register the callback function to call when new data is available.
			 */
			void registerCallback(const CallbackFunction& callback){this->callback = callback;};

		private:
			/**
			 * Configures the modem application.
			 */
			void configure(const labust::xml::ReaderPtr& reader, const std::string& id);
			/**
			 * The receive function start.
			 */
			void start_receive();
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
			boost::shared_ptr<boost::thread> threadPtr;
			/**
			 * MMSerializer factory pointer
			 */
			labust::comms::MMSerializePluginPtr plugin;
			/**
			 * Message decoder
			 */
			labust::comms::MMSerializePtr translator;
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
			 * Message queue to hold packets in multi-packet scenarios.
			 */
			labust::comms::MMSerialize::msgqueue packets;
			/**
			 * Number of modem packets that form a message for the translator.
			 */
			size_t packetNum;
		};

		typedef boost::shared_ptr<UUVModem> UUVModemPtr;
	}
}
/* UUVMODEM_HPP_ */
#endif
