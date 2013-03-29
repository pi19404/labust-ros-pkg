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
*
*  Author: Dula Nad
*  Created: 27.03.2013.
*********************************************************************/
#ifndef MSGENCODER_HPP_
#define MSGENCODER_HPP_

#include <vector>
#include <cstdint>
#include <cmath>



namespace labust
{
	namespace tritech
	{
		/**
		 * The class performs data encoding into acoustic messages to be sent to the diver.
		 */
		class MSGEncoder
		{
			enum{
				pos_init = 1,
				pos,
				pos_msg,
				pos_img,
				pos_def,
				pos_msg_def,
				pos_img_def,
				pos_kml,
				pos_chk,
				pos_msg_chk,
				pos_msg_def_chk,
				pos_img_def_chk
			};

		public:
			/**
			 * Main constructor.
			 */
			MSGEncoder();

			template <size_t precission>
			std::pair<int,int> latlon2(double lat, double lon)
			{

			}


			void packMsg(double depth, double lat, double lon, std::vector<uint8_t>& data)
			{
				const double depthRes = 0.5;

				data[0] = uint8_t(depth /= 0.5);
				double min = (lat - std::floor(lat))*60);
				uint8_t x = int((min - std::floor(min))*10000)%100;
				min = (lon - std::floor(lon))*60);
				uint8_t y = int((min - std::floor(min))*10000)%100;
				data[0] = uint8_t(x/)
			}

			/**
			 * Set the new message.
			 */
			inline void setMsg(const std::vector<uint8_t>& msg)
			{
				this->msg.append(msg);
			}

		private:
			/**
			 * The message buffer.
			 */
			std::vector<uint8_t> msg;
		};
	}
}

/* MSGENCODER_HPP_ */
#endif



