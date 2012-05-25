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
#ifndef UDPSIM_HPP_
#define UDPSIM_HPP_
#include <labust/xml/xmlfwd.hpp>
#include <labust/vehicles/vehiclesfwd.hpp>

#include <boost/asio/ip/udp.hpp>
#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>

namespace labust
{
	namespace simulation
	{
		/**
		 * The class wraps the vehicle simulator in a UDP communication shell. Used
		 * for communication with LabView applications.
		 */
		class UDPSim : boost::noncopyable
		{
		public:
			/**
			 * Main constructor. Configured from the XML configuration file.
			 *
			 * \param reader Pointer to the loaded XML configuration file.
       * \param id Identification class.
			 */
			UDPSim(const labust::xml::ReaderPtr reader, const std::string& id);
			/**
			 * Generic destructor.
			 */
			~UDPSim();

			/**
			 * The method starts the communication.
			 *
			 * \param sync If true the starts blocks otherwise it is non-blocking.
			 *
			 */
			void start(bool sync = true);
			/**
			 * The methods shutdowns the communication.
			 */
			void stop();

		private:
			/**
			 * The method start the message receiving.
			 */
			void start_receive();
			/**
			 * Receive handler.
			 */
			void handle_receive(const boost::system::error_code& error, std::size_t transferred);
			/**
			 * Vehicle plugin
			 */
			labust::vehicles::VehiclePluginPtr plugin;
			/**
			 * UUV simulator pointer
			 */
			labust::vehicles::DriverPtr uuv;
			/**
			 * IO service runner thread.
			 */
			boost::shared_ptr<boost::thread> threadPtr;
			/**
			 * IO service
			 */
			boost::asio::io_service io;
			/**
			 * The udp socket.
			 */
			boost::asio::ip::udp::socket socket_;
			/**
			 * The remote endpoint.
			 */
			boost::asio::ip::udp::endpoint remote;
      /**
       * Remote sender address.
       */
      boost::asio::ip::udp::endpoint remote_sender;
      /**
       * Message buffer
       */
      boost::array<char,64000> buffer;
		};
	}
};

/* UDPSIM_HPP_ */
#endif
