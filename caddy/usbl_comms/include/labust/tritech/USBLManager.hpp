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
#ifndef USBLMANAGER_HPP_
#define USBLMANAGER_HPP_
#include <nodelet/nodelet.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <string>

namespace labust
{
	namespace tritech
	{
		/**
		 * The class implements the Tritech USBL manager that allows specifying which
		 * messages and what data to relay to the modem. It also encodes arrived modem messages.
		 */
		class USBLManager : public nodelet::Nodelet
		{
			/**
			 * The communication states.
			 */
			enum {idle=0,initDiver,waitForReply,transmission};
		public:
			/**
			 * Default constructor.
			 */
			USBLManager();
			/**
			 * Default destructor.
			 */
			~USBLManager();

			/**
			 * Node initialization.
			 */
			void onInit();

		protected:
			/**
			 * Handles arrived USBL navigation messages.
			 */
			void onNavMsg(const auv_msgs::NavSts::ConstPtr nav);
			/**
			 * Handles arrived modem messages.
			 */
			void onIncomingMsg(const std_msgs::String::ConstPtr msg);

			/**
			 * The main runner thread.
			 */
			void run();

			/**
			 * The navigation and incoming data publisher.
			 */
			ros::Publisher outgoing, auto_mode;
			/**
			 * The navigation data subscription.
			 */
			ros::Subscriber navData, incoming;
			/**
			 * The message encoder.
			 */
			//DiverMsg encoder;
			/**
			 * The worker thread.
			 */
			boost::thread worker;
			/**
			 * The current comms state.
			 */
			int state;
			/**
			 * Navigation message flag.
			 */
			bool validNav;
			/**
			 *
			 */
		};
	}
}

/* USBLMANAGER_HPP_ */
#endif



