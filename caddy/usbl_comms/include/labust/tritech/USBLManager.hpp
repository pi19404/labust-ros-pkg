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
#include <labust/tritech/DiverMsg.hpp>

#include <nodelet/nodelet.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <string>
#include <queue>
#include <map>
#include <ctype.h>

namespace labust
{
	namespace tritech
	{
		///\todo Add stream operators.
		struct AsciiInternal
		{
			enum {digit_diff=48, alpha_diff=55};
			enum {alpha_limit_int=36, digit_limit_int=10};
			enum {char_size = 6, zero_char = 63};
			/**
			 * Convert char from ascii to internal encoding.
			 */
			static char ascii2Int(char c)
			{
				if (isalpha(c)) return (toupper(c)-alpha_diff);
				if (isdigit(c)) return (c-digit_diff);
				return zero_char;
			}
			/**
			 * Conver char from internal endoascii to e
			 */
			static char int2Ascii(char c)
			{
				if (c<digit_limit_int) return (c+digit_diff);
				if (c<alpha_limit_int) return (c+alpha_diff);
				return 0;
			}
		};

		/**
		 * The class implements the Tritech USBL manager that allows specifying which
		 * messages and what data to relay to the modem. It also encodes arrived modem messages.
		 *
		 * \todo Should we extract message handlers into different class/classes ?
		 * \todo Extract helper functions for ascii conversions ?
		 */
		class USBLManager : public nodelet::Nodelet
		{
			/**
			 * The communication states.
			 */
			enum {idle=0,initDiver,waitForReply,positionOnly,sendMsg};
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
			/**
			 * State changer.
			 */
			inline void changeState(int next)
			{
				lastState = state;
				state = next;
				NODELET_INFO("Change state: %d -> %d",lastState,next);
			}

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
			 * Handles arrived modem messages.
			 */
			void onIncomingText(const std_msgs::String::ConstPtr msg);
			/**
			 * Handles the USBL timeout.
			 */
			void onUSBLTimeout(const std_msgs::Bool::ConstPtr msg);

			/**
			 * The main runner thread.
			 */
			void run();

			/**
			 * Helper function for initialization.
			 */
			void init_diver();
			/**
			 * Helper function for message sending.
			 */
			void send_msg();
			/**
			 * Helper function for handling incoming text messages.
			 */
			void incoming_txt();

			/**
			 * Helper function for message encoding to int.
			 */
			int msgToInt(int len);
			/**
			 * Helper function for message decoding from int.
			 */
			std::string intToMsg(int len);

			/**
			 * The navigation and incoming data publisher.
			 */
			ros::Publisher outgoing, auto_mode, diverText;
			/**
			 * The navigation data subscription.
			 */
			ros::Subscriber navData, incoming, intext, timeoutNotification;
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
			int state, lastState;
			/**
			 * Navigation message flag.
			 */
			bool validNav;
			/**
			 * The diver message encoder.
			 */
			DiverMsg outgoing_msg, incoming_msg;
			/**
			 * The last sent package.
			 */
			std_msgs::String outgoing_package, incoming_package;
			/**
			 * Text message buffer.
			 */
			std::queue<char> textBuffer;
			/**
			 * The preset messages buffer.
			 */
			std::queue<char> defaultMsgs;
			/**
			 * The kml buffer.
			 */
			std::vector< std::pair<double, double> > kmlBuffer;
			/**
			 * Flag for the turnaround message.
			 */
			bool newMessage;
			/**
			 * Message decoder dispatcher.
			 */
			std::map<int, boost::function<void(void)> > dispatch;
		};
	}
}

/* USBLMANAGER_HPP_ */
#endif



