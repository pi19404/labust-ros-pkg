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
#ifndef RECONFIGUREWIDGET_HPP_
#define RECONFIGUREWIDGET_HPP_

#include <QWidget>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QCheckBox>

#include <string>

namespace labust
{
	namespace gui
	{
		/**
		 * This class implements a dynamically reconfigured widget based on the supplied
		 * update XML.
		 *
		 * \todo Add parameter divison in text boxes.
		 */
		class ReconfigureWidget : public QWidget
		{
			Q_OBJECT
		public:
			/**
			 * Main constructor. Takes a parent widget.
			 *
			 * \param parent The desired parent of the widget.
			 */
			ReconfigureWidget(QWidget* parent = 0);
			/**
			 * Generic destructor.
			 */
			~ReconfigureWidget();

			/**
			 * The method updates the widget contents.
			 *
			 * \param msg The update messages passed to the widget.
			 */
			void update(const std::string& msg);

		signals:
			/**
			 * The Qt signal is emitted on message send commands.
			 */
			void sendCommandRequest(const QString& name, const QString& cmd);

		private slots:
			/**
			 * Handles the send command.
			 */
			void on_SendCommand_clicked();

		private:
			/**
			 * Configure the default widget layout.
			 */
			void configure();

			/**
			 * The update field.
			 */
			QPlainTextEdit* text;
			/**
			 * The return command name.
			 */
			QLineEdit* cmdName;
			/**
			 * Disable receive check.
			 */
			QCheckBox* check;
		};
	}
}



/* RECONFIGUREWIDGET_HPP_ */
#endif
