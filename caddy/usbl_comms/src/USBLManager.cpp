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

#include <geometry_msgs/Point.h>

#include <algorithm>

PLUGINLIB_DECLARE_CLASS(usbl,USBLManager,labust::tritech::USBLManager, nodelet::Nodelet)

using namespace labust::tritech;

USBLManager::USBLManager():
					state(initDiver),
					newMessage(false),
					validNav(false),
					kmlEndValidation(std::make_pair(kmlNotSent,-1)),
					initValidation(),
					timeout(80),
					emptyIterations(0),
					diverOriginLat(0),
					diverOriginLon(0),
					lastKmlIdxSent(-1)
{
	dispatch[DiverMsg::PositionInitAck] = boost::bind(&USBLManager::init_diver,this);

	//Handle multiple combination with one handler
	dispatch[DiverMsg::DefReply] = dispatch[DiverMsg::MsgDefReply] =
			dispatch[DiverMsg::MsgReply] = boost::bind(&USBLManager::incoming_def_txt,this);
	dispatch[DiverMsg::KmlReply] = boost::bind(&USBLManager::incoming_kml,this);
};

USBLManager::~USBLManager(){};

void USBLManager::onInit()
{
	ros::NodeHandle nh = this->getNodeHandle();
	navData = nh.subscribe<auv_msgs::NavSts>("usblFiltered",	1, boost::bind(&USBLManager::onNavMsg,this,_1));
	incoming = nh.subscribe<std_msgs::String>("incoming_data",	1, boost::bind(&USBLManager::onIncomingMsg,this,_1));
	intext = nh.subscribe<std_msgs::String>("usbl_text",	1, boost::bind(&USBLManager::onIncomingText,this,_1));
	inDefaults = nh.subscribe<std_msgs::Int32>("usbl_defaults",	1, boost::bind(&USBLManager::onIncomingDefaults,this,_1));
	inKml = nh.subscribe<std_msgs::Float64MultiArray>("kml_array",	1, boost::bind(&USBLManager::onIncomingKML,this,_1));
	timeoutNotification = nh.subscribe<std_msgs::Bool>("usbl_timeout",	1, boost::bind(&USBLManager::onUSBLTimeout,this,_1));
	forceState = nh.subscribe<std_msgs::Int32>("usbl_force_state",	1, boost::bind(&USBLManager::onIncomingForceState,this,_1));

	outgoing = nh.advertise<std_msgs::String>("outgoing_data",1);
	diverText = nh.advertise<std_msgs::String>("diver_text",1);
	diverDefaults = nh.advertise<std_msgs::Int32>("diver_defaults",1);
	auto_mode = nh.advertise<std_msgs::Bool>("auto_mode",1);
	diverOrigin = nh.advertise<geometry_msgs::Point>("diver_origin",1);
	outCurState = nh.advertise<std_msgs::Int32>("usbl_current_state",1);

	worker = boost::thread(boost::bind(&USBLManager::run,this));
}

void USBLManager::init_diver()
{
	static int sentLat, sentLon;

	if (!validNav)
	{
	   NODELET_ERROR("No valid diver navigation stats received.");
	   return;
	}
	
	//Turn off auto-interrogation	
	std_msgs::Bool data;
	data.data = false;
	auto_mode.publish(data);

	if ((state == initDiver) && (validNav))
	{

		outgoing_package.data = outgoing_msg.toString(DiverMsg::PositionInit);
		sentLat = outgoing_msg.data[DiverMsg::lat];
		sentLon = outgoing_msg.data[DiverMsg::lon];
		diverOriginLat = outgoing_msg.latitude;
		diverOriginLon = outgoing_msg.longitude;
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
			publishDiverOrigin();
			changeState(transmission);
		}
		else
		{
			NODELET_ERROR("Diver initialization failed. Sent (%d, %d) and received (%lld, %lld)",
					sentLat, sentLon, incoming_msg.data[DiverMsg::lat], incoming_msg.data[DiverMsg::lon]);
			//Republish the last message due
			//We need this due to one message lag in the communication
			outgoing.publish(outgoing_package);
		}
	}
}

void USBLManager::incoming_def_txt()
{
	//Current diver message type
	DiverMsg::BitMap& map = DiverMsg::diverMap.at(incoming_msg.data[DiverMsg::type]);

	//If the message has a default
	if (map[DiverMsg::def])
	{
		//Publish the default value
		std_msgs::Int32 mdef;
		mdef.data = incoming_msg.data[DiverMsg::def];

		if (mdef.data == kmlEndValidation.second)
		{
		  NODELET_DEBUG("Kml default validated.");
		  kmlEndValidation.first = kmlSentAndValid;
		}
	
		diverDefaults.publish(mdef);
	}

	//If the message has a default
	if (map[DiverMsg::msg])
	{
		//Decode
		std_msgs::String text;
		text.data = intToMsg(map[DiverMsg::msg]/AsciiInternal::char_size);
		diverText.publish(text);
	}
}

void USBLManager::incoming_kml()
{
	if (kmlVec.size())
	{
		int kmlno = incoming_msg.data[DiverMsg::kmlno];
		if (kmlno < kmlVec.size())
		{
			NODELET_DEBUG("Received kml idx: %d",kmlno);
			bool valid = kmlVec[kmlno].first == incoming_msg.data[DiverMsg::kmlx];
			valid = valid && kmlVec[kmlno].second == incoming_msg.data[DiverMsg::kmly];
			NODELET_INFO("Kml message sent (%d - %d,%d) and received (%d - %lld,%lld).", 
				lastKmlIdxSent, kmlVec[kmlno].first, kmlVec[kmlno].second, kmlno,
				incoming_msg.data[DiverMsg::kmlx], incoming_msg.data[DiverMsg::kmly]);
			
			if (valid)
			{
				kmlValidIdx[kmlno] = kmlSentAndValid;
			}
		
		}
		else
		{
			NODELET_ERROR("Received kmlno=%d from diver "
					"but the kml vector has size %d.",kmlno, kmlVec.size());
		}
	}
	else
	{
		NODELET_INFO("Received new KML point from diver.");
	}
}

std::string USBLManager::intToMsg(int len)
{
	std::string retVal(len,'\0');
	int64_t msg = incoming_msg.data[DiverMsg::msg];
	//NODELET_DEBUG("Decoding text message with size: %d",len);
	//std::cout<<"Message:"<<std::bitset<42>(msg)<<std::endl;
	for (int i=0; i<len; ++i)
	{
		retVal[i] = AsciiInternal::int2Ascii(msg & boost::low_bits_mask_t<AsciiInternal::char_size>::sig_bits);
		msg >>= AsciiInternal::char_size;
	}
	std::reverse(retVal.begin(), retVal.end());
	return retVal;
}

int USBLManager::msgToInt(int len)
{
	int retVal(0);
	//Fill with zero chars if needed.
	while (textBuffer.size() < len) textBuffer.push(AsciiInternal::zero_char);

	for (int i=0; i<len; ++i)
	{
		retVal |= textBuffer.front() & boost::low_bits_mask_t<AsciiInternal::char_size>::sig_bits;
		//NODELET_DEBUG("Message encoding: %c to %d",textBuffer.front(), retVal);
		textBuffer.pop();
		if (i < len-1) retVal <<= AsciiInternal::char_size;
	}

	return retVal;
}

void USBLManager::send_msg()
{
	//This can be replaced with a switch statement
	bool hasMsg(textBuffer.size());
	bool hasKml(kmlBuffer.size() || kmlVec.size());
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
				NODELET_INFO("Send PositionMsgDef: %lld, %lld", outgoing_msg.data[DiverMsg::msg],
						outgoing_msg.data[DiverMsg::def]);
			}
			else
			{
				//Copy the message to the outgoing package
				outgoing_msg.data[DiverMsg::def] = defaultMsgs.front();
				defaultMsgs.pop();
				outgoing_msg.data[DiverMsg::type] = DiverMsg::Position_14Def;
				NODELET_INFO("Send Position_14Def: %lld", outgoing_msg.data[DiverMsg::def]);
			}
		}
		else
		{
			int msglen = DiverMsg::topsideMap[DiverMsg::PositionMsg][DiverMsg::msg]/AsciiInternal::char_size;
			outgoing_msg.data[DiverMsg::msg] = msgToInt(msglen);
			outgoing_msg.data[DiverMsg::type] = DiverMsg::PositionMsg;
			NODELET_INFO("Send PositionMsg: %lld", outgoing_msg.data[DiverMsg::msg]);
		}
	}
	else if (hasKml)
	{
		kml_send();
	}
	else
	{
		//Send position only.
		//outgoing_msg.latitude += 0.001;
		//outgoing_msg.longitude += 0.001;
		NODELET_INFO("Send Position_18: lat=%f, long=%f", outgoing_msg.latitude, outgoing_msg.longitude);
		outgoing_msg.data[DiverMsg::type] = DiverMsg::Position_18;
	}

	//Send message
	outgoing_package.data = outgoing_msg.toString();
	outgoing.publish(outgoing_package);
	changeState(waitForReply);
}

///\todo Simplify this message logic when we have time, it can be rewritten in a nicer shape using and automaton.
void USBLManager::kml_send()
{
	static int kml_send_size = 1 << DiverMsg::topsideMap.at(DiverMsg::PositionKml)[DiverMsg::kmlno];
	//If none are scheduled for sending
	if (kmlVec.size() == 0)
	{
		int i=0;

		while (!kmlBuffer.empty() && i<kml_send_size)
		{
			++i;
			kmlVec.push_back(kmlBuffer.front());
			kmlValidIdx.push_back(0);
			kmlBuffer.pop();
		}
		kmlEndValidation.first = kmlNotSent;
	}

	//Find first non sent point, increment idx in for declaration
	int idx=0;
	for(int i=0; i<kmlValidIdx.size(); ++i, ++idx)
	{
		if (kmlValidIdx[i] == kmlNotSent)
		{
			kmlValidIdx[idx] = kmlSent;
		 	break;
		}
	}

	//If all sent try to find a non-validated
	if (idx == kmlValidIdx.size())
	{
		idx = 0;
		for(int i=0; i<kmlValidIdx.size(); ++i, ++idx) if (kmlValidIdx[i] != kmlSentAndValid) break;

		//If all validated
		if (idx == kmlValidIdx.size())
		{
			if (kmlEndValidation.first == kmlNotSent || kmlEndValidation.first == kmlWaitValidation)
			{
				//Send KML end default message
				outgoing_msg.data[DiverMsg::type] = DiverMsg::Position_14Def;
				outgoing_msg.data[DiverMsg::def] = kmlBuffer.empty()?DiverMsg::endOfKML:
					DiverMsg::nextKMLSet;
				kmlEndValidation.first = kmlSent;
				kmlEndValidation.second = outgoing_msg.data[DiverMsg::def];

				if (kmlBuffer.empty())
				{
					NODELET_INFO("Sending end kml.");
					lastKmlIdxSent = kml_send_size+1;
				}
				else
				{
					NODELET_INFO("Sending next kml.");
					lastKmlIdxSent = kml_send_size;
				}
			}
			else if (kmlEndValidation.first == kmlSent)
			{
				outgoing_msg.data[DiverMsg::type] = DiverMsg::Position_18;
				NODELET_INFO("Sending position while wait for kml reply.");
				kmlEndValidation.first = kmlWaitValidation;
			}
			else if (kmlEndValidation.first == kmlSentAndValid)
			{
				//Clear vector
				kmlVec.clear();
				kmlValidIdx.clear();
				//recurse 
				if (kmlEndValidation.second == DiverMsg::nextKMLSet) 
				{
					kml_send();
				}
				else
				{
					outgoing_msg.data[DiverMsg::type] = DiverMsg::Position_18;
				}
			}
			return;
		}
		else
		{
			int kmlUnvalidated = 0;
			//count unvalidated
			for(int i=0; i<kmlValidIdx.size(); ++i) if (kmlValidIdx[i] != kmlSentAndValid) ++kmlUnvalidated;
			//For the last non-validated kml value we have to send some
			//random message to try and get a validation
			if (kmlUnvalidated == 1)
			{
				//In order to validate the last message we have to do a pull with some message
				//Send a position update message as a pull message
				if (kmlValidIdx[idx] != kmlWaitValidation)
				{
					//Do not send a kml message
					outgoing_msg.data[DiverMsg::type] = DiverMsg::Position_18;
					//Send a position message
					kmlValidIdx[idx] = kmlWaitValidation;
					return;
				}
				//but if we sent a position message and no validation occured resend the last kml
				else
				{
					//Resend the last unvalidated kml
					kmlValidIdx[idx] = kmlSent;
				}
			}
			else
			{
				for(int i=0; i<kmlValidIdx.size(); ++i)
				{ 
				  if (kmlValidIdx[i] == kmlSent) kmlValidIdx[i] = kmlNotSent;
				}
	
				idx = 0;
				for(int i=0; i<kmlValidIdx.size(); ++i, ++idx)
				{			
					if (kmlValidIdx[i] == kmlNotSent)
					{
						kmlValidIdx[idx] = kmlSent;
		 				break;
					}
				}
			}
		}
	}

	//Send KML message if we did not return till now from the function
	outgoing_msg.data[DiverMsg::type] = DiverMsg::PositionKml;
	outgoing_msg.data[DiverMsg::kmlno] = idx;
	outgoing_msg.data[DiverMsg::kmlx] = kmlVec[idx].first;
	outgoing_msg.data[DiverMsg::kmly] = kmlVec[idx].second;
	lastKmlIdxSent = idx;
	NODELET_INFO("Sent PositionKml: %d, (%lld,%lld)",idx,
			outgoing_msg.data[DiverMsg::kmlx],
			outgoing_msg.data[DiverMsg::kmly]);
}

///\todo Switch all automata like this to a boost state chart or something to avoid switch
///\todo Switch the manager to async triggered instead of having a run method.
void USBLManager::run()
{
	double freq;
	ros::NodeHandle ph("~");
	ph.param("rate",freq,10.0);
	ros::Rate rate(freq);
	//8 sec. 
	timeout = 10*freq;
	std_msgs::String package;

	while (ros::ok())
	{
		switch (state)
		{
		case idle: break;
		case waitForReply:
			if (++emptyIterations > timeout) resendLastPackage();
			break;
		case initDiver:
			this->init_diver();
			break;
		case transmission:
			//Send the different messages and navigation data
			NODELET_INFO("\nManager transmit:");
			this->send_msg();
			break;
		default:
			NODELET_ERROR("Unknown state.");
			state = idle;
		}

		//NODELET_INFO("Manager running.");

		//Reset the turn-around flag
		newMessage = false;
		std_msgs::Int32Ptr curstate(new std_msgs::Int32());
		curstate->data = state;
		outCurState.publish(curstate);
		publishDiverOrigin();
		

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
	
	//Reset the empty iterations counter.
	emptyIterations = 0;

	try
	{
		incoming_msg.fromString(msg->data);
		incoming_package = *msg;
		int msg_type = incoming_msg.data[DiverMsg::type];
		//NODELET_INFO("Disptach message: %d",msg_type);
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

	if (state == waitForReply) changeState(lastState);
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
		resendLastPackage();
	}
}

void USBLManager::resendLastPackage()
{
		//resend last package
		//repack the message with possibly new navigation data.
		NODELET_INFO("Timeout - resending last package. Iterations: %d",emptyIterations);
		outgoing_package.data = outgoing_msg.toString();
		outgoing.publish(outgoing_package);

		//Reset the empty iterations counter.
		emptyIterations = 0;
}


void USBLManager::onIncomingDefaults(const std_msgs::Int32::ConstPtr msg)
{
	//Do some checking on the default messages
	defaultMsgs.push(msg->data);
}

void USBLManager::onIncomingForceState(const std_msgs::Int32::ConstPtr msg)
{
	//Do some checking on the default messages
	if (msg->data < lastStateNum)
	{
		this->lastState = msg->data;
	}
	else
	{
		ROS_ERROR("Unknown state %d",msg->data);
	}
}

void USBLManager::onIncomingKML(const std_msgs::Float64MultiArray::ConstPtr msg)
{
	int len = msg->data.size();
	if (len % 2 != 0)
	{
		--len;
		NODELET_WARN("The kml array should have a even lengths. Pairs of {lat0,lon0,...,latN,lonN}");
	}

	LatLon2Bits llEncoder;
	static int kmlbits = DiverMsg::topsideMap.at(DiverMsg::PositionKml)[DiverMsg::kmlx];
	for (int i=0; i<len; i+=2)
	{
		llEncoder.convert(msg->data[i],msg->data[i+1], kmlbits);
		kmlBuffer.push(std::make_pair(llEncoder.lat,llEncoder.lon));
	}
}

void USBLManager::publishDiverOrigin()
{
	geometry_msgs::PointPtr msg(new geometry_msgs::Point());
	msg->x = diverOriginLat;
	msg->y = diverOriginLon;
	diverOrigin.publish(msg);
}

