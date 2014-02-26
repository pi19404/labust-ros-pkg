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
#ifndef SPATIALNODE_HPP_
#define SPATIALNODE_HPP_
#include <labust/navigation/SpatialMessages.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <string>

namespace labust
{
	namespace navigation
	{
		/**
		 * The class implements the Spatial INS driver.
		 * \todo Implement additional messages
		 */
		class SpatialNode
		{
			enum {lrc=0, packet_id, packet_length, crc0, crc1, headerSize};

		public:
			/**
			 * Main constructor
			 */
			SpatialNode();
			/**
			 * Generic destructor.
			 */
			~SpatialNode();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

		private:
			/**
			 * Handle the header detection.
			 */
			void onHeader(const boost::system::error_code& e, std::size_t size);
			/**
			 * Handle the incoming data stream.
			 */
			void onData(const boost::system::error_code& e, std::size_t size);

			/**
			 * Configure the Spatial device
			 */
			void configureSpatial();
			/**
			 * Update status of the device.
			 */
			void statusUpdate(uint16_t systemStatus, uint16_t filterStatus);

			/**
			 * Send spatial command.
			 */
			void sendToDevice(uint8_t id, uint8_t len, const std::string& data);
			/**
			 * Handle system state message.
		   */
			void onSystemStatePacket(boost::archive::binary_iarchive& data);
			/**
			 * Handle the euler covariance message.
			 */
			void onVec3fPacket(vec3f& vec, boost::archive::binary_iarchive& data);
			/**
			 * Handle the velocity covariance message.
			 */
			void onAckPacket(boost::archive::binary_iarchive& data);


			/**
			 * The serial port setup helper method.
			 */
			bool setup_port();
			/**
			 * The start receive helper function.
			 */
			void start_receive();

			/**
			 * The ROS publisher.
			 */
			ros::Publisher imu, gps, vel;

			/**
			 * Hardware i/o service.
			 */
			boost::asio::io_service io;
			/**
			 * The serial input port.
			 */
			boost::asio::serial_port port;
			/**
			 * The main operation thread.
			 */
			boost::thread runner;
			/**
			 * The input buffer.
			 */
			boost::asio::streambuf buffer;
			/**
			 * The header ring buffer.
			 */
			std::vector<uint8_t> ringBuffer;
			/**
			 * The data handler function prototype.
			 */
			typedef boost::function<void(boost::archive::binary_iarchive&)> HandlerFunction;
			/**
			 * The handler map.
			 */
			typedef std::map<int,HandlerFunction> HandlerMap;
			/**
			 * The handler map.
			 */
			HandlerMap handler;

			/**
			 * The euler and velocity covariance.
			 */
			vec3f eulerCov, velCov;
		};
	}
}

/* SPATIALNODE_HPP_ */
#endif
