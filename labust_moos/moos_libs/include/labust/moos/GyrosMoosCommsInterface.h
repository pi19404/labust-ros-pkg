/* 
 * File:   GyrosMoosCommsInterface.h
 * Author: tomi
 *
 * Created on February 14, 2011, 3:35 PM
 */

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
 
#ifndef _GYROSMOOSCOMMSINTERFACE_H
#define	_GYROSMOOSCOMMSINTERFACE_H

#include <labust/comms/GyrosCommsInterface.hpp>
#include <labust/moos/MoosConfig.hpp>

#include <MOOSLIB/MOOSApp.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/noncopyable.hpp>

namespace labust
{
    namespace comms
    {

        class GyrosMoosCommsInterface : public GyrosCommsInterface, CMOOSApp, boost::noncopyable
        {
        public:
            ///
            /// Constructor that reads MOOS configuration from a preloaded xml file /xmlreader)
			/// xml reader must be set to the right <config> node before
            ///
            /// \param reader xml reader with preloaded and preset config file, mandatory
			/// \param configToUse name of configuration to use, if empty, the first one will be used
            ///
            GyrosMoosCommsInterface(const labust::xml::Reader &reader, std::string configToUse = "");

            ///
            /// Constructor that reads MOOS configuration from an xml file on disk
            /// finds the first element <commsConfig type="moos"> with the configToUse name (if provided)
            ///
            /// \param configPath path to config file, mandatory
			/// \param configToUse name of configuration to use, if empty, the first one will be used
            ///
            GyrosMoosCommsInterface(const std::string& configPath, std::string configToUse = "");
            
            virtual ~GyrosMoosCommsInterface();

            /**
             * PASSIVE or ACTIVE usage
             * Sends data through MOOS
             * data is passed as a GYROSWriter object \see GyrosWriter.hpp
             * MOOS variable the data is sent to is determined by the label of gyros object
             * If gyros has no label, output variable is "[process name]_OUT";
             *
             * \param data reference to GYrosWriter with data to send
             * \param wait true if function should block until the data is sent, false to return immediately
             * \return COMERRORS::ComError in case of error, 0 if everything is ok
             */
            commerrors::CommError Send(const labust::xml::GyrosWriter &data, bool wait = false);

            /**
             * PASSIVE or ACTIVE usage
             * Sends data through MOOS
             * data is passed as a GYROSWriter object vector \see GyrosWriter.hpp
             * MOOS variable the data is sent to is determined by the label of gyros object
             * If gyros has no label, output variable is "[process name]_OUT";
             *
             * \param data reference to GYrosWriter vector with data to send
             * \param wait true if function should block until the data is sent, false to return immediately
             * \return COMERRORS::ComError in case of error, 0 if everything is ok
             */
            commerrors::CommError Send(const std::vector<labust::xml::GyrosWriter> &data, bool wait = false);

            /**
             * PASSIVE
             * This methods allows to receive data from the comms buffer.
             * in case the system has to initiate communication
             * Stores all received data in the GYROSreader vector as multiple objects
             * If lock is set, the call blocks until data is available for receiving, otherwise exits with an empty vector
             * NAme of variable data was received from is stored as label of the gyros object
             *
             * \param data vector of gyros objects to hold new data
             * \param wait true if the call should block until data is available, false to return error code (noData)
             * \return COMERRORS::ComError in case of error, 0 if everything is ok
             */
            commerrors::CommError Receive(std::vector<labust::xml::GyrosReader> &data, bool wait = false);

            /**
             * Returns pointer to MOOS client
             *
             * \return pointer to MOOS client
             */
            const void* GetCommObject(void);

            /**
             * ACTIVE
             * Allows registration of object which will handle incomming messages from MOOS
             *
             * \param obj pointer to object on which callbacks will be executed
             */
            virtual void RegisterCallbackObject(CommEntity *entity);

            /**
             * sets callbackobject to NULL
             */
            virtual void UnRegisterCallbackObject();

            /**
             * MOOSApp overrides
             * \see MOOSAPP.h
             */
            bool OnNewMail(MOOSMSG_LIST &NewMail);
            bool Iterate();

            bool OnConnectToServer();
            bool OnStartUp();
			bool OnDisconnectFromServer();
            
        private:
            MoosConfig config;
            std::vector<labust::xml::GyrosWriter> messagesToSend;
            std::vector<labust::xml::GyrosReader> messagesReceived;

            boost::thread commsThread;
            boost::condition_variable dataSentSignal, dataReceivedSignal;
            boost::mutex locker;
        };
    }
}
#endif	/* _GYROSMOOSCOMMSINTERFACE_H */

