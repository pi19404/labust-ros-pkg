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
#ifndef XMLMESSAGEEXCHANGE_HPP_
#define XMLMESSAGEEXCHANGE_HPP_
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <string>

namespace labust
{
	namespace gui
	{
		/**
		 * The class implements a mediation object for connection routing between the
		 * communication layer and DynRec GUI application.
		 */
		class XMLMessageExchange
		{
		public:
			typedef boost::shared_ptr<XMLMessageExchange> Ptr;
			typedef boost::function<bool (const std::string&, const std::string&) > MessageCallback;
			typedef boost::function<bool (const std::string&) > RegisterCallback;

			class CommsCommands
			{
			public:
				typedef boost::shared_ptr<CommsCommands> Ptr;
				typedef boost::shared_ptr<const CommsCommands> CPtr;

				inline void addRegisterVariableCallback(const RegisterCallback& callback)
				{
					this->onRegister = callback;
				}
				inline void addUnRegisterVariableCallback(const RegisterCallback& callback)
				{
					this->onUnRegister = callback;
				}
				inline void addSendCommandCallback(const MessageCallback& callback)
				{
					this->onSendCommand = callback;
				}

				inline bool registerVariable(const std::string& name) const
				{
					return this->onRegister(name);
				}
				inline bool unRegisterVariable(const std::string& name) const
				{
					return this->onUnRegister(name);
				}

				inline bool sendCommand(const std::string& name, const std::string& data) const
				{
					return this->onSendCommand(name, data);
				}

			private:
				MessageCallback onSendCommand;
				RegisterCallback onRegister,onUnRegister;
			};

			class GUICommands
			{
			public:
				typedef boost::shared_ptr<GUICommands> Ptr;
				typedef boost::shared_ptr<const GUICommands> CPtr;

				inline void addNewMessageCallback(const MessageCallback& callback)
				{
					this->onNewMessage = callback;
				}

				inline void newMessage(const std::string& name, const std::string& data) const
				{
					this->onNewMessage(name,data);
				}
			private:
				MessageCallback onNewMessage;
			};

			template <class GUI, class Comms>
			static void connect(GUI* const gui, Comms* const comms)
			{
				comms->setGUICommands(gui->getGUICommands());
				gui->setCommsCommands(comms->getCommsCommands());
			};
		};
	};
};

/* XMLMESSAGEEXCHANGE_HPP_ */
#endif
