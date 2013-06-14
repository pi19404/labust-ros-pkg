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
 *  Created on: 28.05.2013.
 *  Author: Dula Nad
 *********************************************************************/
#ifndef CNRRemoteRADIO_HPP_
#define CNRRemoteRADIO_HPP_
#include <labust/preprocessor/mem_serialized_struct.hpp>
#include <labust/tools/StringUtilities.hpp>
#include <cart2/RadioModemConfig.h>

#include <dynamic_reconfigure/server.h>

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <auv_msgs/NavSts.h>
#include <ros/ros.h>

#include <sstream>

namespace labust
{
	namespace control
	{
		/**
		 * The class that implements the CART side of the CNR remote station control protocol.
		 */
		class CNRRemoteRadio
		{
			enum {sync_length=3, chksum_size = 2};
			//Estimates
			enum {id_field = 3, data1_field = 4, data2_field=8, mode_field = 12, launch_field=4};
			enum {stopbit, startbit, manualbit, automaticbit, remotebit};
			enum {cart =0, bart=1, station=2};
		public:
			/**
			 * Main constructor
			 */
			CNRRemoteRadio();
			/**
			 * Main deconstructor
			 */
			~CNRRemoteRadio();
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
			 * Handle the estimates.
			 */
			void onStateHat(const auv_msgs::NavSts::ConstPtr& estimate);
			/**
			 * Handle the measurements.
			 */
			void onCurrentMode(const std_msgs::Int32::ConstPtr& mode);
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
			 * Modem timeout detection.
			 */
			void onTimeout();
			/**
			 * Send the reply message.
			 */
			void reply();
			/**
			 * Send the buoy reply message.
			 */
			void replyBuoy();
			/**
			 * Send the buoy reply message.
			 */
			void dummyRequest();

			/**
			 * The ROS node handles.
			 */
			ros::NodeHandle nh,ph;
			/**
			 * The last arrived message.
			 */
			ros::Time lastModemMsg;
			/**
			 * The timeout length.
			 */
			double timeout, currYaw, yawInc, currLat, currLon;
			/**
			 * The publishers.
			 */
			ros::Publisher joyOut, launched, hlMsg, posOut;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber extPoint, joyIn, stateHat, stateMeas, curMode;
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
			 * The data protector.
			 */
			boost::mutex cdataMux,clientMux;
			/**
			 * The asio streambuffer.
			 */
			std::vector<uint8_t> buffer;
			/**
			 * The sync buffer.
			 */
			uint8_t ringBuffer[sync_length];
			/**
			 * The service client.
			 */
			ros::ServiceClient client;
			/**
			 * My id.
			 */
			int32_t id;
			/**
			 * Dummy requester.
			 */
			bool doDummyRequest;
			/**
			 * Last mode field.
			 */
			uint8_t lastmode;
		};
	}
}
/* CNRREMOTERADIO_HPP_ */
#endif
