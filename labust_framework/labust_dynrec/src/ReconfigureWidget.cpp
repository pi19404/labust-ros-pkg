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
#include <labust/gui/ReconfigureWidget.hpp>
#include <QPushButton>
#include <QLabel>
#include <QDebug>
#include <QHBoxLayout>
#include <QVBoxLayout>

using namespace labust::gui;

ReconfigureWidget::ReconfigureWidget(QWidget* parent):
		QWidget(parent){this->configure();};

ReconfigureWidget::~ReconfigureWidget(){}

void ReconfigureWidget::update(const std::string& msg)
{
	if (!check->isChecked())
	{
		std::string mmsg(msg);
		size_t cur = mmsg.find(">");
		while (cur != std::string::npos)
		{
			if (mmsg.at(++cur) != '\n') mmsg.insert(cur,"\n");
			cur = mmsg.find(">",cur);
		}

		text->setPlainText(mmsg.c_str());
	}
}

void ReconfigureWidget::on_SendCommand_clicked()
{
	emit sendCommandRequest(cmdName->text(), text->toPlainText());
}

void ReconfigureWidget::configure()
{
	QPushButton* button(new QPushButton);
	button->setText(tr("Send command"));
	connect(button,SIGNAL(clicked()),this,SLOT(on_SendCommand_clicked()));
	QLabel* label(new QLabel(tr("Command Name:")));
	cmdName = new QLineEdit("cmd");
	check = new QCheckBox();
	check->setText(tr("Disable receive"));

	QHBoxLayout* hlayout(new QHBoxLayout);
	hlayout->addWidget(label);
	hlayout->addWidget(cmdName);
	hlayout->addWidget(button);
	hlayout->addWidget(check);

	text = new QPlainTextEdit();
	text->setPlainText(tr("Waiting for update..."));
	QVBoxLayout* layout(new QVBoxLayout());
	layout->addWidget(text);
	layout->addLayout(hlayout);

	this->setLayout(layout);
}

