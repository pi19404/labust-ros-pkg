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
#ifndef DYNRECMOOSAPP_HPP_
#define DYNRECMOOSAPP_HPP_
#include <MOOSLIB/MOOSApp.h>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <string>

namespace labust
{
	namespace moos
	{
		/**
		 * This class implements the MOOS application for the dynamic reconfigure node.
		 */
		class DynRecMoosApp : public CMOOSApp
		{
		public:
			/**
			 * Generic MOOSApp constructor.
			 */
			DynRecMoosApp();
			/**
			 * Generic destructor.
			 */
			~DynRecMoosApp();

			/**
			 * Inherited method from CMOOSApp.
			 */
			bool OnConnectToServer();

		protected:
			/**
			 * Inherited method from CMOOSApp.
			 */
			bool Iterate();
			/**
			 * Inherited method from CMOOSApp.
			 */
			bool OnNewMail(MOOSMSG_LIST& NewMail);
			/**
			 * Inherited method from CMOOSApp.
			 */
			bool OnStartUp();

		private:
			/**
			 * Perform registration of MOOS variables.
			 */
			bool registerMoosVariables();
			/**
			 * Read mission parameters from the configuration file.
			 */
			bool readMissionParameters();

			/**
			 * Control variables for MOOS connection and startup
			 */
			bool connected, startUP;
		};
	}
}
/* DYNRECMOOSAPP_HPP_ */
#endif
