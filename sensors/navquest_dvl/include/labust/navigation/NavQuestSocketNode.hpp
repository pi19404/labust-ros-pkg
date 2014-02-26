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
#ifndef NAVQUESTSOCKETNODE_HPP_
#define NAVQUESTSOCKETNODE_HPP_

#include <labust/navigation/NavQuestMessages.hpp>
#include <labust/navigation/BeamPublisher.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

namespace labust
{
	namespace navigation
	{
		/**
		 * The class implements the Nav Quest DVL acquisition.
		 * \todo Implement additional messages
		 * \todo Rewrite the legacy DVLDataClass into a more compact parser.
		 * \todo Add calibration synchonization with IMU sensor calibration
		 * \todo Publish IMU data.
		 * \todo Add sound velocity subscription to correct speed measurements.
		 */
		class NavQuestSocketNode
		{
			enum {u=0,v,w};

		public:
			/**
			 * Main constructor
			 */
			NavQuestSocketNode();
			/**
			 * Generic destructor.
			 */
			~NavQuestSocketNode();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

		protected:
			/**
			 * Handle the incoming data stream.
			 */
			void onDvlData(const boost::system::error_code& e, std::size_t size);

			/**
			 * The socket setup helper method.
			 */
			bool setup_socket();
			//bool setup_port();
		
			/**
			 * The start receive helper function.
			 */
			void start_receive();
			/**
			 * Helper function for message decoding.
			 */
			void setup_messaging();
			/**
			 * Helper topic publisher.
			 */
			void setup_publishers();
			/**
			 * Helper function for header testing.
			 */
			bool test_header(const std::string& match , const std::string& stream);
			/**
			 * Publish DVL data.
			 */
			void publishDvlData(const NQRes& data);
			/**
			 * Condition the data before publishing.
			 */
			void conditionDvlData(const NQRes& data);

			typedef BeamPublisher<4> NavQuestBP;
			/**
			 * The beam publishers.
			 */
			std::map<std::string, NavQuestBP::Ptr > beam_pub;
			/**
			 * The speed publishers.
			 */
			std::map<std::string, TwistPublisher::Ptr > speed_pub;
			/**
			 * The speed publishers.
			 */
			ros::Publisher imuPub, lock, altitude;;
			/**
			 * The transform broadcaster.
			 */
			tf2_ros::TransformBroadcaster broadcast;
			/**
			 * Fixed rotation flag.
			 */
			bool useFixed;
			/**
			 * Fixed rotation value.
			 */
			double base_orientation;
			/**
			 * Magnetic declination.
			 */
			double magnetic_declination;

			/**
			 * Hardware i/o service.
			 */
			boost::asio::io_service io;
			/**
			 * The serial input port.
			 */
			boost::asio::ip::tcp::socket socket;
			//boost::asio::serial_port port;
			/**
			 * The main operation thread.
			 */
			boost::thread runner;
			/**
			 * The input buffer.
			 */
			boost::asio::streambuf buffer;
		};
	}
}



/* NAVQUESTSOCKETNODE_HPP_ */
#endif
