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
 *  Created on: 06.05.2013.
 *  Author: Dula Nad
 *********************************************************************/
#ifndef TOPSIDERADIO_HPP_
#define TOPSIDERADIO_HPP_
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <cart2/RadioModemConfig.h>

#include <dynamic_reconfigure/server.h>

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(radio),TopsideModemData,
		(float, surgeForce)
		(float, torqueForce)
		(double, lat)
		(double, lon)
		(float, radius)
		(float, surge)
		(int32_t, mode)
		(uint8_t, launch))


		namespace labust
		{
	namespace control
	{
		/**
		 * The class implements the topside radio modem node.
		 * \todo Split into two policies.
		 */
		class TopsideRadio
		{
			enum {sync_length=6};
		public:
			/**
			 * Main constructor
			 */
			TopsideRadio();
			/**
			 * Main deconstructor
			 */
			~TopsideRadio();
			/**
			 * Initialize and setup the manager.
			 */
			void onInit();
			/**
			 * Start the radio node.
			 */
			void start();

		private:
			/**
			 * Dynamic reconfigure callback.
			 */
			void dynrec_cb(cart2::RadioModemConfig& config, uint32_t level);
			/**
			 * Handle the joystick input.
			 */
			void onJoy(const sensor_msgs::Joy::ConstPtr& joy);
			/**
			 * Handle the external target point.
			 */
			void onExtPoint(const geometry_msgs::PointStamped::ConstPtr& isLaunched);
			/**
			 * Helper function.
			 */
			void populateDataFromConfig();
			/**
			 * Helper function
			 */
			void local2LatLon(double x, double y);
			/**
			 * Start the receiving thread.
			 */
			void start_receive();
			/**
			 * Handle incoming modem data.
			 */
			void onSync(const boost::system::error_code& error, const size_t& transferred);
			/**
			 * Handle incoming modem data.
			 */
			void onIncomingData(const boost::system::error_code& error, const size_t& transferred);

			/**
			 * The ROS node handles.
			 */
			ros::NodeHandle nh,ph;
			/**
			 * The publishers.
			 */
			ros::Publisher joyOut, launched, hlMsg;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber extPoint, joyIn;

			/**
			 * The io service.
			 */
			boost::asio::io_service io;
			/**
			 * The serial port
			 */
			boost::asio::serial_port port;
			/**
			 * The io service thread.
			 */
			boost::thread iorunner;
			/**
			 * The dynamic reconfigure server.
			 */
			dynamic_reconfigure::Server<cart2::RadioModemConfig> server;
			/**
			 * The last dynamic reconfiguration.
			 */
			cart2::RadioModemConfig config;
			/**
			 * Exchanged data.
			 */
			labust::radio::TopsideModemData data;
			/**
			 * The data protector.
			 */
			boost::mutex dataMux;
			/**
			 * The asio streambuffer.
			 */
			boost::asio::streambuf sbuffer;
			/**
			 * The sync buffer.
			 */
			std::string ringBuffer;
			/**
			 * Location flag.
			 */
			bool isTopside;
			/**
			 * The service client.
			 */
			ros::ServiceClient client;
			/**
			 * The frame transformer.
			 */
			double originLat, originLon;
		};
	}
}
/* TOPSIDERADIO_HPP_ */
#endif
