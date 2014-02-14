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
*  Created: 02.04.2013.
*********************************************************************/
#ifndef USBLNODELET_HPP_
#define USBLNODELET_HPP_
#include <labust/tritech/tritechfwd.hpp>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <string>

namespace labust
{
	namespace tritech
	{
		/**
		 * The class implements the Tritech USBL acquisition nodelet.
		 */
		class USBLNodelet : public nodelet::Nodelet
		{
		public:
			/**
			 * Default constructor.
			 */
			USBLNodelet();
			/**
			 * Default destructor.
			 */
			~USBLNodelet();

			/**
			 * Node initialization.
			 */
			void onInit();
			/**
			 * The main run method.
			 */
			void run();
			/**
			 * Cleanly stops the main method but leaves on
			 * the USBL connection.
			 */
			void stop();

		protected:
			/**
			 * Handles arrived USBL navigation messages.
			 */
			void onAttMsg(labust::tritech::TCONMsgPtr tmsg);
			/**
			 * Handles arrived USBL navigation messages.
			 */
			void onNavMsg(labust::tritech::TCONMsgPtr tmsg);
			/**
			 * Handles other USBL messages.
			 */
			void onTCONMsg(labust::tritech::TCONMsgPtr tmsg);
			/**
			 * Handles outgoing messages requests.
			 */
			void onOutgoingMsg(const std_msgs::String::ConstPtr msg);
			/**
			 * Handles outgoing messages requests.
			 */
			void onAutoMode(const std_msgs::Bool::ConstPtr mode);
			/**
			 * Send one USBL encoded package.
			 */
			void sendUSBLPkg();
			/**
			 * Ping timeout.
			 */
			int ping_timeout;

			/**
			 * The USBL device.
			 */
			TCPDevicePtr usbl, attitude;
			/**
			 * The USBL address.
			 */
			std::string address;
			/**
			 * The USBL port.
			 */
			int port;
			/**
			 * Last message from USBL.
			 */
			ros::Time lastUSBL;
			/**
			 * The outgoing message.
			 */
			std::string msg_out;
			/**
			 * The USBL status.
			 */
			bool usblBusy;
			/**
			 * The lock condition variable.
			 */
			boost::condition_variable usblCondition;

			/**
			 * The data and condition mux.
			 */
			boost::mutex dataMux, pingLock;
			/**
			 * The worker thread.
			 */
			boost::thread worker, guard;

			/**
			 * The navigation and incoming data publisher.
			 */
			ros::Publisher navPub, dataPub, usblTimeout, attRaw, attData;
			/**
			 * The outgoing data subscription.
			 */
			ros::Subscriber dataSub, opMode;
			/**
			 * USBL frame transformation broadcaster.
			 */
			tf2_ros::TransformBroadcaster frameBroadcast;
			/**
			 * Auto interrogate mode.
			 */
			bool autoMode;
		};
	}
}

/* USBLNODELET_HPP_ */
#endif



