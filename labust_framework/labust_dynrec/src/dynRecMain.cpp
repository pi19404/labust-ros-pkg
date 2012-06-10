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
#include <QApplication>
#include <labust/gui/DynRecMainWindow.hpp>
#include <labust/moos/DynRecMoosApp.hpp>
#include <labust/gui/XMLMessageExchange.hpp>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

int main(int argc, char* argv[])
try
{
  const char* sMissionFile = "config/configure.moos";
  const char* sMOOSName = "pDynRec";

  switch(argc)
  {
    case 3:
      sMOOSName = argv[2];
    case 2:
      sMissionFile = argv[1];
    default:break;
  }

  QApplication app(argc, argv);

  labust::moos::DynRecMoosApp moosapp;
  labust::gui::DynRecMainWindow mainwindow;
  mainwindow.show();
  labust::gui::XMLMessageExchange::connect(&mainwindow,&moosapp);

  boost::thread t(boost::bind(&labust::moos::DynRecMoosApp::Run,&moosapp,sMOOSName,sMissionFile));
  app.exec();

  moosapp.RequestQuit();
  t.join();

  return 0;
}
catch (std::exception& e)
{
	std::cerr<<e.what()<<std::endl;
}
catch (...)
{
	std::cerr<<"Unknown error."<<std::endl;
}



