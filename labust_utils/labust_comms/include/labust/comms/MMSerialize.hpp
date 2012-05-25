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
#ifndef MMSERIALIZE_HPP_
#define MMSERIALIZE_HPP_
#include <labust/plugins/Factory.hpp>
#include <labust/plugins/DLLoad.hpp>

#include <boost/shared_ptr.hpp>

#include <string>
#include <deque>

namespace labust
{
	namespace comms
	{
		/**
		 * This is the interface for message serialization.
		 */
		class MMSerialize
		{
		public:
			/**
			 * Message typedef.
			 */
			typedef std::deque<std::string> msgqueue;
			/**
			 * Generic virtual destructor.
			 */
			virtual ~MMSerialize(){};
			/**
			 * This method takes the received modem message in string format and converts it into a
			 * user readable XML string.
			 *
			 * \param mmsg Encoded modem message
			 * \param xmlstr Address of the decoded xml_string
			 *
			 * \return True if the message was readable and False otherwise.
			 */
			virtual bool decode(const msgqueue& mmsg, std::string* const xmlstr) = 0;
			/**
			 * This method takes a user readable XML string and converts it into a modem message string.
			 *
			 * \param xmlstr Address of the desired xml_string
			 * \param mmsg Encoded modem message queue.
			 *
			 * \return True if the encoding was successful, false otherwise.
			 */
			virtual bool encode(const std::string& xmlstr, msgqueue* const mmsg) = 0;
			/**
			 * This method returns the number of serial packets that are needed to assemble
			 * the whole message.
			 * Default packet length is assumed one.
			 */
			virtual size_t packetNum(){return 1;};
			/**
			 * This method specifies if a binary or formated protocol is required.
			 * Defaults to a formated protocol type.
			 */
			virtual bool isBinary(){return false;};
			/**
			 * This method is used by binary plugins to specify what read length is required next.
			 */
			virtual size_t nextReadLength(){return 0;}
		};
	}
}

/* MMSERIALIZE_HPP_ */
#endif
