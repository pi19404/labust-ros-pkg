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
#ifndef BENCHRADIO_HPP_
#define BENCHRADIO_HPP_
#include <labust/preprocessor/mem_serialized_struct.hpp>

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <auv_msgs/BodyForceReq.h>
#include <ros/ros.h>

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(radio),BenchModemData,
		(float, surgeForce)
		(float, torqueForce))

namespace labust
{
	namespace radio
	{
		typedef boost::array<float,4> quat;
	}
}
BOOST_CLASS_IMPLEMENTATION(labust::radio::quat , boost::serialization::primitive_type)

PP_LABUST_DEFINE_BOOST_SERIALIZED_STRUCT_CLEAN((labust)(radio),BenchModemDataRet,
		(float, surgeAch)
		(float, torqueAch)
		(char, windupS)
		(char, windupT)
		(quat, orientation)
		(double, lat)
		(double, lon))

namespace labust
{
	namespace control
	{
		/**
		 * The class implements the bench radio modem node. For testing without WLAN.
		 * \todo Split into two policies.
		 * \todo Add in ros utils a generic conversion of different message, i.e. NavSts,
		 * to a vector or named map.
		 * \todo Replace synchonization with read_until
		 */
		class BenchRadio
		{
			enum {sync_length=6, Bench_package_length=8, cart_package_length=42};
		public:
			/**
			 * Main constructor
			 */
			BenchRadio();
			/**
			 * Main deconstructor
			 */
			~BenchRadio();
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
			 * Handle the tau in.
			 */
			void onTauIn(const auv_msgs::BodyForceReq::ConstPtr& tauIn);
			/**
			 * Handle the external target point.
			 */
			void onImu(const sensor_msgs::Imu::ConstPtr& imu);
			void onGps(const sensor_msgs::NavSatFix::ConstPtr& gps);
			/**
			 * Handle the estimates.
			 */
			void onTauAchIn(const auv_msgs::BodyForceReq::ConstPtr& tau);
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
			 * The publishers.
			 */
			ros::Publisher tauOut, imuOut, tauAch,gpsOut;
			/**
			 * The subscribed topics.
			 */
			ros::Subscriber tauIn, imuIn, tauAchIn, gpsIn;
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
			 * Exchanged data.
			 */
			labust::radio::BenchModemData data;
			/**
			 * The return data.
			 */
			labust::radio::BenchModemDataRet cdata;
			/**
			 * The data protector.
			 */
			boost::mutex cdataMux;
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
			bool isBench;
		};
	}
}
/* BENCHRADIO_HPP_ */
#endif
