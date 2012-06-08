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
#ifndef DYNRECMAINWINDOW_HPP_
#define DYNRECMAINWINDOW_HPP_
#include <labust/gui/XMLMessageExchange.hpp>
#include <labust/xml/xmlfwd.hpp>

#include <boost/thread/mutex.hpp>

#include <QMainWindow>

#include <map>
#include <string>

namespace Ui
{
	class DynRecMainWindow;
	typedef boost::shared_ptr<DynRecMainWindow> DynRecMainWindowPtr;
};

namespace labust
{
	namespace gui
	{
		/**
		 * The class implements the main windows of the dynamic reconfigure application
		 * for the LABUST navcon framework.
		 *
		 * \todo Add tab termination with double-click
		 * \todo Add command sending
		 * \todo Add focus to new tab
		 */
		class DynRecMainWindow : public QMainWindow
		{
			Q_OBJECT

			typedef std::map<std::string, QWidget*> VariableMap;
			typedef labust::gui::XMLMessageExchange::CommsCommands CommsCommands;
			typedef labust::gui::XMLMessageExchange::GUICommands GUICommands;
		public:
			/**
			 * Generic constructor.
			 */
			DynRecMainWindow();
			/**
			 * Main constructor. Configures the main window using the XML configuration file.
			 *
			 * \param reader The pointer to the XML configuration file.
			 * \param id The configuration identifier.
			 */
			DynRecMainWindow(const labust::xml::ReaderPtr reader, const std::string id = "");
			/**
			 * Generic destructor.
			 */
			~DynRecMainWindow();

			GUICommands::CPtr getGUICommands();
			/**
			 * The method connects the mediation object with this main window.
			 *
			 * \param mediator The mediator object that handles request and notifications.
			 */
			inline void setCommsCommands(CommsCommands::CPtr mediator)
			{
				this->mediator = mediator;
			}

		private slots:
			/**
			 * The command handler.
			 */
			void on_newVariableName_returnPressed();

		private:
			/**
			 * The generic configuration.
			 */
			void configure();
			/**
			 * The XML configuration.
			 */
			void configure(const labust::xml::ReaderPtr reader, const std::string id);

			/**
			 * The method handles incoming messages. The message is thread safe.
			 *
			 * \param name The message name.
			 * \param data The message data.
			 */
			bool onNewMessage(const std::string& name, const std::string& data);
			/**
			 * The method handles GUI adjusments when a new variable is added.
			 *
			 * \param The name of the new variable.
			 */
			QWidget* makeNewTab();

			/**
			 * The GUI initial configuration.
			 */
			Ui::DynRecMainWindowPtr gui;
			/**
			 * The mediator object for the communication layer connection.
			 */
			CommsCommands::CPtr mediator;

			/**
			 * The message list.
			 */
			VariableMap messageMap;
			/**
			 * The onNewMessage mutex.
			 */
			boost::mutex newMessageSync;
		};
	}
}



#endif /* DYNRECMAINWINDOW_HPP_ */
