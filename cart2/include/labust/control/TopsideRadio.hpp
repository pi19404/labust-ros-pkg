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
#include <labust/tools/StringUtilities.hpp>
#include <cart2/RadioModemConfig.h>
#include <cart2/ImuInfo.h>
#include <labust/control/crc16.h>

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
/**
 * The topside transmitted message.
 *
 * force, torque - (-100,100)
 */
PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(radio),TopsideModemData,
		(int8_t, surgeForce)
		(int8_t, torqueForce)
		(int32_t, lat)
		(int32_t, lon)
		(uint8_t, radius)
		(uint8_t, surge)
		(int8_t, yaw)
		(uint8_t, mode)
		(uint8_t, launch)
		(uint8_t, mode_update))

namespace labust
{
	namespace radio
	{
		typedef boost::array<int16_t,4> stateVec;
		typedef boost::array<int16_t,3> measVec;
	}
}
BOOST_CLASS_IMPLEMENTATION(labust::radio::stateVec , boost::serialization::primitive_type)
BOOST_CLASS_IMPLEMENTATION(labust::radio::measVec , boost::serialization::primitive_type)

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(radio),CARTModemData,
		(int32_t, origin_lat)
		(int32_t, origin_lon)
		(uint8_t, mode)
		(measVec, state_meas)
		(stateVec, state_hat)
		(measVec, sf_state)
		(int16_t, portRPM)
		(int16_t, stbdRPM)
		(uint8_t, voltage)
		(uint8_t, temp))

namespace labust
{
	namespace control
	{
		/**
		 * The class implements the topside radio modem node.
		 * \todo Split into two policies.
		 * \todo Add in ros utils a generic conversion of different message, i.e. NavSts,
		 * to a vector or named map.
		 * \todo Replace synchonization with read_until
		 */
		class TopsideRadio
		{
			enum {sync_length=2, chksum_size = 2, topside_package_length=16+chksum_size, cart_package_length=35+chksum_size};
			//Estimates
			enum {x=0,y,psi,u};
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
			 * Handle the estimates.
			 */
			void onStateHat(const auv_msgs::NavSts::ConstPtr& estimate);
			/**
			 * Handle the measurements.
			 */
			void onStateMeas(const auv_msgs::NavSts::ConstPtr& meas);
			void onSFMeas(const auv_msgs::NavSts::ConstPtr& meas);
			void onCartInfo(const cart2::ImuInfo::ConstPtr& info);
			/**
			 * Handle the measurements.
			 */
			void onCurrentMode(const std_msgs::Int32::ConstPtr& mode);
			/**
			 * Helper function.
			 */
			void populateDataFromConfig();
			/**
			 * Helper function.
			 */
			void local2LatLon(double x, double y);
			/**
			 * Helper function.
			 */
			void sendCData();
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
			 * Helper function for checksum calculation.
			 */
			template <class MsgType>
			uint8_t calculateChecksum(MsgType& chdata)
			{
				std::ostringstream chk;
				boost::archive::binary_oarchive chkSer(chk, boost::archive::no_header);
				chkSer << chdata;
				return labust::tools::getChecksum(
						reinterpret_cast<const uint8_t*>(chk.str().data()), chk.str().size());
			}
			/**
			 * Helper function for checksum calculation.
			 */
			template <class MsgType>
			int calculateCRC16(MsgType& chdata)
			{
				std::ostringstream chk;
				boost::archive::binary_oarchive chkSer(chk, boost::archive::no_header);
				chkSer << chdata;
				return compute_crc16(	reinterpret_cast<const char*>(chk.str().data()), chk.str().size());
			}
			/**
			 * Modem timeout detection.
			 */
			void onTimeout();

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
			double timeout;
			/**
			 * The publishers.
			 */
			ros::Publisher joyOut, launched, hlMsg, stateHatPub, stateMeasPub, info;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber extPoint, joyIn, stateHat, stateMeas, curMode, sfFrame, cartInfo;
			/**
			 * The transform listener.
			 */
			tf::TransformListener listener;
			/**
			 * The transform broadcaster.
			 */
			tf::TransformBroadcaster broadcaster;
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
			 * The return data.
			 */
			labust::radio::CARTModemData cdata;
			/**
			 * The data protector.
			 */
			boost::mutex dataMux, cdataMux,clientMux;
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
			bool isTopside, twoWayComms;
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
