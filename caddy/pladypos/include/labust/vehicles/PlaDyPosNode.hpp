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
#ifndef PLADYPOSNODE_HPP_
#define PLADYPOSNODE_HPP_
#include <labust/vehicles/ScaleAllocation.hpp>
#include <pladypos/ThrusterMappingConfig.h>

#include <auv_msgs/BodyForceReq.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <Eigen/Dense>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

namespace labust
{
	namespace vehicles
	{
		/**
		 * The PlaDyPos ROS node for allocation, control and monitoring of the surface platform.
		 *
		 * \todo When the stable version is reached spin off a general templated or
		 * dynamically loaded vehicle driver
		 */
		class PlaDyPosNode
		{
			/**
			 * The scaling constants for the driver sensors.
			 */
			static const float sscale[6];
			/**
			 * Sensor enumeration.
			 */
			enum {current0, current1, current2, current3, currentConv, voltage};
		public:
			/**
			 * Main constructor.
			 */
			PlaDyPosNode();

			/**
			 * General destructor.
			 */
			~PlaDyPosNode();

			void configure(ros::NodeHandle& nh, ros::NodeHandle& ph);

		protected:
			/**
			 * The desired force and torque subscriber.
			 */
			ros::Subscriber tau;
			/**
			 * The achieved force and torque publisher.
			 */
			ros::Publisher tauAch, diag, info;

			/**
			 * Helper method to generate the driver msg.
			 */
			void driverMsg(const int n[4]);
			/**
			 * Publishes the diagnostics.
			 */
			void pubDiagnostics();

			/**
			 * Handles the arrived force and torque requests.
			 */
			void onTau(const auv_msgs::BodyForceReq::ConstPtr tau);
			/**
			 * Safety function.
			 */
			void safetyTest();
			/**
			 * Dynamic reconfigure service.
			 */
			void dynrec(pladypos::ThrusterMappingConfig& config, uint32_t level);
			/**
			 * Handles the arrived message.
			 */
			void onReply(const boost::system::error_code& error, const size_t& transferred);
			/**
			 * Starts the message receive.
			 */
			void start_receive();
			/**
			 * The safety test thread.
			 */
			boost::thread safety;
			/**
			 * The output mutex.
			 */
			boost::mutex serialMux;
			/**
			 * The last TAU.
			 */
			ros::Time lastTau;
			/**
			 * The timeout.
			 */
			double timeout;

			/**
			 * Dynamic reconfigure server.
			 */
			dynamic_reconfigure::Server<pladypos::ThrusterMappingConfig> server;
			/**
			 * The external revolution control flag.
			 */
			bool revControl;

			/**
			 * The asio IO service.
			 */
			boost::asio::io_service io;
			/**
			 * The pladypos serial port.
			 */
			boost::asio::serial_port port;
			/**
			 * The io service thread.
			 */
			boost::thread iorunner;
			/**
			 * The input buffer.
			 */
			boost::asio::streambuf sbuffer;
			/**
			 * The sensor measurements
			 */
			boost::array<float,6> sensors;
			/**
			 * The last revolutions.
			 */
			int lastRevs[4];

			/**
			 * Allocation matrix and maximum force, torque.
			 */
			Eigen::Matrix<float, 3,4> B;
			/**
			 * The scale allocator.
			 */
			ScaleAllocation allocator;
		};
	}
}
/* PLADYPOSNODE_HPP_ */
#endif
