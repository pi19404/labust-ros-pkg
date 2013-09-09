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
#include <labust/tritech/USBLManager.hpp>
#include <pluginlib/class_list_macros.h>

#include <boost/integer/integer_mask.hpp>

PLUGINLIB_DECLARE_CLASS(usbl,USBLManager,labust::tritech::USBLManager, nodelet::Nodelet)

using namespace labust::tritech;

USBLManager::USBLManager():
			state(initDiver),
			newMessage(false),
			validNav(false)
{
	dispatch[DiverMsg::PositionInitAck] = boost::bind(&USBLManager::init_diver,this);
	dispatch[DiverMsg::Msg] = boost::bind(&USBLManager::incoming_txt,this);
};

USBLManager::~USBLManager(){};

void USBLManager::onInit()
{
	ros::NodeHandle nh = this->getNodeHandle();
	navData = nh.subscribe<auv_msgs::NavSts>("usblFiltered",	1, boost::bind(&USBLManager::onNavMsg,this,_1));
	incoming = nh.subscribe<std_msgs::String>("incoming_data",	1, boost::bind(&USBLManager::onIncomingMsg,this,_1));
	intext = nh.subscribe<std_msgs::String>("text",	1, boost::bind(&USBLManager::onIncomingText,this,_1));
	timeoutNotification = nh.subscribe<std_msgs::Bool>("usbl_timeout",	1, boost::bind(&USBLManager::onUSBLTimeout,this,_1));
	outgoing = nh.advertise<std_msgs::String>("outgoing_data",1);
	diverText = nh.advertise<std_msgs::String>("diver_txt",1);
	auto_mode = nh.advertise<std_msgs::Bool>("auto_mode",1);

	worker = boost::thread(boost::bind(&USBLManager::run,this));
}

void USBLManager::init_diver()
{
	static int sentLat, sentLon;

	if (!validNav) NODELET_ERROR("No valid diver navigation stats received.");

	if ((state == initDiver) && (validNav))
	{
		std_msgs::Bool data;
		data.data = false;
		auto_mode.publish(data);

		outgoing_package.data = outgoing_msg.toString(DiverMsg::PositionInit);
		sentLat = outgoing_msg.data[DiverMsg::lat];
		sentLon = outgoing_msg.data[DiverMsg::lon];
		outgoing.publish(outgoing_package);

		//Change to transmission state
		changeState(waitForReply);
	}
	else if ((lastState == initDiver) && (state == waitForReply))
	{
		//incoming_msg.fromString<DiverMsg::AutoDiver>(incoming_package.data, DiverMsg::PositionInitAck);

		if ((sentLat == incoming_msg.data[DiverMsg::lat]) && (sentLon == incoming_msg.data[DiverMsg::lon]))
		{
			NODELET_INFO("Diver initialization successful.");
			changeState(transmission);
		}
		else
		{
			NODELET_INFO("Diver initialization failed. Sent (%d, %d) and received (%lld, %lld)",
					sentLat, sentLon, incoming_msg.data[DiverMsg::lat], incoming_msg.data[DiverMsg::lon]);
			changeState(initDiver);
		}
	}
}

void USBLManager::incoming_txt()
{
	//Decode
	int size = DiverMsg::diverMap[DiverMsg::Msg][DiverMsg::msg]/AsciiInternal::char_size;
	std_msgs::String text;
	text.data = intToMsg(size);
	diverText.publish(text);
}

std::string USBLManager::intToMsg(int len)
{
	std::string retVal(len,'\0');
	int64_t msg = incoming_msg.data[msg];
	for (int i=0; i<len; ++i)
	{
		retVal[i] = msg & boost::low_bits_mask_t<AsciiInternal::char_size>::sig_bits;
		msg >> AsciiInternal::char_size;
	}

	return retVal;
}

int USBLManager::msgToInt(int len)
{
	NODELET_INFO("Encode message.");
	int retVal(0);
	//Fill with zero chars if needed.
	while (textBuffer.size() < len) textBuffer.push(AsciiInternal::zero_char);

	for (int i=0; i<len; ++i)
	{
		retVal |= textBuffer.front() & boost::low_bits_mask_t<AsciiInternal::char_size>::sig_bits;
		NODELET_INFO("Message encoding: %c to %d",textBuffer.front(), retVal);
		textBuffer.pop();
		if (i < len-1) retVal <<= AsciiInternal::char_size;
	}

	return retVal;
}

void USBLManager::send_msg()
{
		//This can be replaced with a switch statement
		bool hasMsg(textBuffer.size());
		bool hasKml(kmlBuffer.size());
		bool hasDefault(defaultMsgs.size());

		if (hasDefault || hasMsg)
		{
			if (hasDefault)
			{
				if (hasMsg)
				{
					outgoing_msg.data[DiverMsg::def] = defaultMsgs.front();
					defaultMsgs.pop();
					int msglen = DiverMsg::topsideMap[DiverMsg::PositionMsgDef][DiverMsg::msg]/AsciiInternal::char_size;
					outgoing_msg.data[DiverMsg::msg] = msgToInt(msglen);
					outgoing_msg.data[DiverMsg::type] = DiverMsg::PositionMsgDef;
				}
				else
				{
					//Copy the message to the outgoing package
					outgoing_msg.data[DiverMsg::def] = defaultMsgs.front();
					defaultMsgs.pop();
					outgoing_msg.data[DiverMsg::type] = DiverMsg::Position_14Def;
				}
			}
			else
			{
				NODELET_INFO("Send message.", outgoing_msg.latitude, outgoing_msg.longitude);
				int msglen = DiverMsg::topsideMap[DiverMsg::PositionMsg][DiverMsg::msg]/AsciiInternal::char_size;
				outgoing_msg.data[DiverMsg::msg] = msgToInt(msglen);
				outgoing_msg.data[DiverMsg::type] = DiverMsg::PositionMsg;
			}
		}
		else if (hasKml)
		{
			//Process and send the KML data.
		}
		else
		{
			//Send position only.
			outgoing_msg.latitude += 0.001;
			outgoing_msg.longitude += 0.001;
			NODELET_INFO("Send position only: lat=%f, long=%f", outgoing_msg.latitude, outgoing_msg.longitude);
			outgoing_msg.data[DiverMsg::type] = DiverMsg::Position_18;
		}

		//Send message
		outgoing_package.data = outgoing_msg.toString();
		outgoing.publish(outgoing_package);
		changeState(waitForReply);
}

///\todo Switch all automata like this to a boost state chart or something to avoid switch
///\todo Switch the manager to async triggered instead of having a run method.
void USBLManager::run()
{
	ros::Rate rate(10);
	std_msgs::String package;

	while (ros::ok())
	{
		switch (state)
		{
		case idle: break;
		case waitForReply: break;
		case initDiver:
			this->init_diver();
			break;
		case transmission:
			//Send the different messages and navigation data
			this->send_msg();
			break;
		default:
			ROS_ERROR("Unknown state.");
			state = idle;
		}

		NODELET_INFO("Manager running.");

		//Reset the turn-around flag
		newMessage = false;

		rate.sleep();
		ros::spinOnce();
	}
}

void USBLManager::onNavMsg(const auv_msgs::NavSts::ConstPtr nav)
{
	NODELET_DEBUG("Received nav message.");
	validNav = true;
	outgoing_msg.latitude = nav->global_position.latitude;
	outgoing_msg.longitude = nav->global_position.longitude;
	outgoing_msg.depth = nav->position.depth;
}

void USBLManager::onIncomingMsg(const std_msgs::String::ConstPtr msg)
{
	NODELET_INFO("Received modem message with type: %d",DiverMsg::testType(msg->data));
	
	try
	{
		incoming_msg.fromString(msg->data);
		incoming_package = *msg;
		int msg_type = incoming_msg.data[DiverMsg::type];
		if (dispatch.find(msg_type) != dispatch.end())
		{
			dispatch[msg_type]();
		}
		else
		{
			NODELET_ERROR("No handler for message type %d",msg_type);
		}
	}
	catch (std::exception& e)
	{
		NODELET_ERROR("Exception caught on incoming msg: %s",e.what());
	}

	if (state == waitForReply && lastState == transmission) changeState(transmission);
	if (state == waitForReply && lastState == initDiver) changeState(initDiver);
}

void USBLManager::onIncomingText(const std_msgs::String::ConstPtr msg)
{
	for (int i=0; i<msg->data.size(); ++i)
	{
		textBuffer.push(AsciiInternal::ascii2Int(msg->data[i]));
	}
}

void USBLManager::onUSBLTimeout(const std_msgs::Bool::ConstPtr msg)
{
	if  (msg->data && state == waitForReply)
	{
		//resend last package
		//repack the message with possibly new navigation data.
		NODELET_INFO("Resending last package.");
		outgoing_package.data = outgoing_msg.toString();
		outgoing.publish(outgoing_package);
	}
}
