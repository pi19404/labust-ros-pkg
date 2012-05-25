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
#ifndef SERIALHANDLER_HPP_
#define SERIALHANDLER_HPP_

#include <boost/regex.hpp>
#include <boost/asio.hpp>

namespace labust
{
	namespace comms
	{
		class FormatedHandler
		{
		public:
			typedef boost::function<void (std::string)> CallbackFunction;





		protected:

			inline void start_receive(boost::asio::serial_port& port)
			{
				boost::asio::async_read_until(port, sbuffer, boost::regex("\n"),this->handler);
			}

			void handleInput(const boost::system::error_code& error, const size_t& transferred)
			{
				std::istream is(&sbuffer);
				std::string line;
			  std::getline(is, line);
			  incoming.push_back(line);

			  if (incoming.size() == packetNum)
			  {
			  	if (!translator->decode(incoming, &xmlstr))
			  	{
			  		std::cout<<"Failed to decode message:"<<line<<std::endl;
			  		incoming.pop_front();
			  	}
			  	else
			  	{
			  		for(size_t i=0; i<packetNum; ++i){incoming.pop_front();}
			  		this->callback(xmlstr);
			  	}
			  }

				start_receive();
			}

			/**
			 * Stream buffer
			 */
			boost::asio::streambuf sbuffer;
			/**
			 * Callback functor.
			 */
			CallbackFunction callback;
		};
	}
}


/* SERIALHANDLER_HPP_ */
#endif
